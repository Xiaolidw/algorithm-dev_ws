#!/usr/bin/env python3
"""
YOLOv8 颜色识别 + 3D 定位节点

订阅 /camera/image_raw，用 YOLOv8 对红色和蓝色方块做目标检测，
弹窗显示检测框，终端打印中心像素坐标，
并通过 TF + 相机内参发布 PoseStamped（3D 坐标）。

模型: 省赛训练的 best.pt (yolov8)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point  # 注册 PointStamped 的 TF 变换
import cv2
import numpy as np
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# 方块真实边长（米）
BLOCK_SIZE = 0.06

# 调试图片保存路径
DEBUG_DIR = "/tmp/color_detector_debug"

# YOLO 模型路径
MODEL_PATH = os.path.join(get_package_share_directory("tools_demo"), "models", "vision", "best.pt")
# 置信度阈值（省赛模型跨场景迁移，稍放宽）
CONF_THRESHOLD = 0.25


class ColorDetector(Node):
    """YOLOv8 目标检测 + 3D 定位节点"""

    def __init__(self):
        super().__init__("color_detector")

        # ---- 2D 检测 ----
        self.bridge = CvBridge()
        self.frame_count = 0
        self.has_display = "DISPLAY" in os.environ

        # ---- YOLO 模型 ----
        self.get_logger().info(f"加载 YOLO 模型: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)

        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 10
        )

        # ---- 3D 定位 ----
        # 相机内参 K (3×3 矩阵)，收到 /camera/camera_info 后赋值
        self.K = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self._camera_info_cb, 10
        )

        # TF 变换：camera_link → base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布 3D 目标位姿（每个检测到的方块发一条）
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/target_pose", 10)

        self.get_logger().info(
            f"ColorDetector 启动 (阶段2: 2D检测+3D定位) | "
            f"显示器: {'有' if self.has_display else '无(仅保存图片)'}"
        )
        if not self.has_display:
            os.makedirs(DEBUG_DIR, exist_ok=True)
            self.get_logger().info(f"调试图片将保存到 {DEBUG_DIR}")

    # ================================================================
    # 相机内参回调 — 拿到 K 矩阵（9 个数字的"相机说明书"）
    # ================================================================
    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is not None:
            return  # 内参不变，只取一次
        # msg.k 是长度为 9 的 float64 数组，按行优先排列
        self.K = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(
            f"相机内参已获取 | fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
            f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}"
        )

    # ================================================================
    # 图像回调 — 每收到一帧执行一次
    # ================================================================
    def callback(self, msg: Image):
        # ---- 0. 图像转换 ----
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        self.frame_count += 1

        # ---- 1. YOLO 推理 ----
        results = self.model(frame, verbose=False, conf=CONF_THRESHOLD)[0]
        all_targets = []
        for box in results.boxes:
            cls_name = self.model.names[int(box.cls[0])]
            color = "red" if "red" in cls_name else "blue"
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            all_targets.append((color, cx, cy, x1, y1, x2 - x1, y2 - y1))

        # ---- 2. 日志输出（每 30 帧打印一次）----
        if self.frame_count % 30 == 0:
            if all_targets:
                self.get_logger().info(
                    f"检测到 {len(all_targets)} 个目标: "
                    + ", ".join(
                        f"{t[0]}({t[1]},{t[2]})" for t in all_targets
                    )
                )
            else:
                self.get_logger().info("无目标")

        # ---- 3. 3D 定位 ----
        if self.K is not None and all_targets:
            for target in all_targets:
                self._publish_3d_pose(target, msg.header.stamp)

        # ---- 4. 可视化 ----
        self._visualize(frame, all_targets)

    # ================================================================
    # 3D 定位 — 像素坐标 → 相机坐标 → TF → base_link 坐标
    #
    # 步骤：
    #   A. 针孔模型：像素(cx,cy) + 深度 Z → 光学坐标系 (X,Y,Z)
    #   B. 旋转：光学坐标系 (x右,y下,z前) → camera_link (x前,y左,z上)
    #   C. TF：camera_link → base_link（自动查 TF 树）
    #   D. 发布 PoseStamped
    # ================================================================
    def _publish_3d_pose(self, target, stamp):
        color, cx, cy, x, y, w, h = target

        # ---- A. 用像素框大小估算深度 ----
        # 原理：初中相似三角形 — 像素大小/焦距 = 真实大小/距离
        fx = self.K[0, 0]          # x方向焦距（像素单位）
        fy = self.K[1, 1]          # y方向焦距
        u0 = self.K[0, 2]          # 光心 x 像素坐标
        v0 = self.K[1, 2]          # 光心 y 像素坐标

        # 用框的较大边估算（更稳定），避免除零
        pixel_size = max(w, h, 1)
        Z = fx * BLOCK_SIZE / pixel_size

        # ---- B. 针孔模型反投影（光学坐标系） ----
        # 光学坐标系: x=右, y=下, z=前（相机视线方向）
        X_opt = (cx - u0) * Z / fx     # 目标在相机右方多少米
        Y_opt = (cy - v0) * Z / fy     # 目标在相机下方多少米

        # ---- C. 光学坐标系 → camera_link 坐标系 ----
        # camera_link: x=前, y=左, z=上（和 base_link 方向一致）
        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_link"
        point_cam.header.stamp = stamp
        point_cam.point.x = float(Z)       # 光学前方 = camera_link 前方
        point_cam.point.y = float(-X_opt)  # 光学右方 = camera_link 左方
        point_cam.point.z = float(-Y_opt)  # 光学下方 = camera_link 上方

        # ---- D. TF 变换: camera_link → base_link ----
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "camera_link", rclpy.time.Time()
            )
            point_base = do_transform_point(point_cam, t)
        except TransformException as e:
            self.get_logger().debug(f"TF 变换失败 ({color}): {e}")
            return

        # ---- E. 发布 PoseStamped ----
        pose = PoseStamped()
        pose.header = point_base.header
        pose.header.frame_id = "base_link"
        pose.pose.position = point_base.point
        pose.pose.orientation.w = 1.0  # 无旋转

        self.pose_pub.publish(pose)

        # 每 30 帧打一次 3D 坐标
        if self.frame_count % 30 == 0:
            pos = point_base.point
            self.get_logger().info(
                f"  3D [{color}] base_link坐标: "
                f"({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                f"距离≈{Z:.3f}m"
            )

    # ================================================================
    # 可视化 — 弹窗显示检测框
    # ================================================================
    def _visualize(self, frame, targets):
        # 检测框
        display = frame.copy()
        color_map = {"red": (0, 0, 255), "blue": (255, 0, 0)}  # BGR

        for name, cx, cy, x, y, w, h in targets:
            c = color_map.get(name, (0, 255, 0))
            cv2.rectangle(display, (x, y), (x + w, y + h), c, 2)
            cv2.circle(display, (cx, cy), 3, c, -1)
            cv2.putText(
                display, f"{name}", (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2,
            )

        if self.has_display:
            cv2.imshow("color_detector", display)
            cv2.waitKey(1)
        elif self.frame_count % 30 == 0:
            cv2.imwrite(f"{DEBUG_DIR}/frame_{self.frame_count:06d}.jpg", display)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
