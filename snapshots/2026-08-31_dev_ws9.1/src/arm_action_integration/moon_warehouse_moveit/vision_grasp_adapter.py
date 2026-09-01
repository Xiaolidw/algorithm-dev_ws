#!/usr/bin/env python3
"""Bridge /perception/target_pose into manipulator action sequences.

Subscribes to YOLO's 3D target pose, computes matching arm joints, and
runs one complete pick→place cycle by composing existing ROS primitives:

    1. Move arm (via FollowJointTrajectory directly)
    2. Open/close gripper  (via ExecuteManipulation action)
    3. Attach/detach object (via /ATTACHLINK /DETACHLINK services)

RViz-friendly markers are published on /arm/marker showing grasp/place
positions so you can verify computation visually.
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from linkattacher_msgs.srv import AttachLink, DetachLink
from moon_warehouse_interfaces.action import ExecuteManipulation
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped


class VisionGraspAdapter(Node):
    """Subscribe to vision → send arm commands → run pick/place pipeline."""

    # ── tuning knobs ──────────────────────────────────────────────────
    GRASP_OFFSET = 0.40          # metres to approach along X axis
    PLACE_X = 3.5                # drop-zone x  (map frame)
    PLACE_Y = 2.5                # drop-zone y  (map frame)
    ARM_DURATION_S = 8.0         # matches fixed_manipulation.yaml

    # ── known joint templates ─────────────────────────────────────────
    JOINT_INIT = [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0]
    JOINT_BEND = [0.0, 1.2, 1.17, 0.0, -0.3, 0.0]

    def __init__(self):
        super().__init__("vision_grasp_adapter")
        self._cb_group = ReentrantCallbackGroup()

        # Clients
        self._manip_client = ActionClient(
            self, ExecuteManipulation, "/manipulation/execute",
            callback_group=self._cb_group,
        )
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            callback_group=self._cb_group,
        )

        # Subscriptions
        self._target_sub = self.create_subscription(
            PoseStamped, "/perception/target_pose",
            self._on_target, 1, callback_group=self._cb_group,
        )

        # Publishers
        self._marker_pub = self.create_publisher(
            Marker, "/arm/marker", 10)
        self._status_pub = self.create_publisher(
            String, "/manipulation/status", 10)

        self._latest_target = None
        self.get_logger().info("VisionGraspAdapter started.")

        # Show initial markers
        self._publish_marker(self.PLACE_X, self.PLACE_Y, 0.0,
                             self.MARKER_ID_PLACE(), "place_zone")

    MARKER_NS = "grasp_pipeline"

    def MARKER_ID_GRASP(self):
        return 100

    def MARKER_ID_PLACE(self):
        return 101

    # ==================================================================
    # Perception callback
    # ==================================================================

    def _on_target(self, msg: PoseStamped):
        self._latest_target = msg
        self.get_logger().info_once(
            f"Target received: ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")

    # ==================================================================
    # Public API — call these from another node or ros2 action client
    # ==================================================================

    async def run_pick_at_target(self, dry_run: bool = False):
        """Full cycle: open → move_to_target → close → attach → lift_home."""
        if self._latest_target is None:
            self.get_logger().error("No target perceived yet.")
            self._publish_status("no_target")
            return

        self._publish_status("pick_start")

        if dry_run:
            await self._dry_pick()
        else:
            await self._do_pick(self._latest_target)

        self._publish_status("pick_done")

    async def run_place_at_zone(self, dry_run: bool = False):
        """Move to place zone → open → detach → home."""
        if dry_run:
            await self._dry_place()
        else:
            await self._do_place()

        self._publish_status("place_done")

    # ==================================================================
    # Pick implementation
    # ==================================================================

    async def _do_pick(self, target: PoseStamped):
        # 1. Open gripper
        await self._exec_manip("open", "")

        # 2. Compute arm joints from target position
        joints = self._compute_joints(target)

        # 3. Move arm
        await self._send_arm_joints(joints, "move_to_pick_pose")

        # Update grasp marker for visual verification
        self._publish_marker(
            target.pose.position.x, target.pose.position.y,
            target.pose.position.z, self.MARKER_ID_GRASP(), "grasp")

        # 4. Close gripper
        await self._exec_manip("close", "")

        # 5. Attach
        await self._attach_object("red_cube_1", True)

        # 6. Lift / return home
        await self._send_arm_joints(self.JOINT_INIT, "lift_home")

    async def _do_place(self):
        # 1. Move to place zone joint pose
        await self._send_arm_joints(self.JOINT_BEND, "move_to_place_pose")

        # 2. Open gripper
        await self._exec_manip("open", "")

        # 3. Detach
        await self._attach_object("red_cube_1", False)

        # 4. Return home
        await self._send_arm_joints(self.JOINT_INIT, "return_home")

    async def _dry_pick(self):
        """Step-through without touching hardware."""
        for stage in ("validate_target", "dry_open", "dry_move",
                      "dry_close", "dry_attach", "dry_lift"):
            self.get_logger().info(f"[DRY] {stage}")
            await self._sleep(0.2)

    async def _dry_place(self):
        for stage in ("dry_move_place", "dry_open",
                      "dry_detach", "dry_home"):
            self.get_logger().info(f"[DRY] {stage}")
            await self._sleep(0.2)

    # ==================================================================
    # Joint computation — world XY → arm joints
    # ==================================================================

    def _compute_joints(self, target: PoseStamped):
        """Linear blend between init and bend based on target distance.

        When target is far (> 2 m from origin) → full bend.
        When target is close (< 0.5 m) → stay near init.
        Linearly interpolates in between.
        """
        tx = target.pose.position.x
        ty = target.pose.position.y
        dist = math.sqrt(tx * tx + ty * ty)

        # Clamp: very close targets get a reduced angle to avoid singularities
        t = min(max((dist - 0.5) / 2.0, 0.0), 1.0)

        return [a + (b - a) * t
                for a, b in zip(self.JOINT_INIT, self.JOINT_BEND)]

    # ==================================================================
    # Low-level helpers
    # ==================================================================

    async def _exec_manip(self, op: str, obj_id: str):
        """Send a simple operation via ExecuteManipulation action."""
        if not self._manip_client.wait_for_server(timeout_sec=10):
            raise RuntimeError(f"Manipulation action unavailable: {op}")

        goal = ExecuteManipulation.Goal()
        goal.operation = op
        goal.object_id = obj_id

        send_fut = self._manip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            raise RuntimeError(f"Manipulation goal rejected: {op}")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        if not result.success:
            raise RuntimeError(f"'{op}' failed: {result.message}")

    async def _send_arm_joints(self, joints, stage="move_arm"):
        """Send a single-point joint trajectory directly to the arm controller."""
        if not self._arm_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("Arm controller unavailable.")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        whole = int(self.ARM_DURATION_S)
        frac = (self.ARM_DURATION_S - whole) * 1e9
        point.time_from_start = Duration(sec=whole, nanosec=int(frac))
        goal.trajectory.points = [point]

        send_fut = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            raise RuntimeError(f"Arm trajectory rejected: {stage}")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Arm failed ({stage}): error_code={res.error_code}")

    async def _attach_object(self, object_id: str, do_attach: bool):
        srv_name = "/ATTACHLINK" if do_attach else "/DETACHLINK"
        req_type = AttachLink.Request if do_attach else DetachLink.Request

        client = self.create_client(req_type, srv_name)
        if not await self._svc_wait(client):
            verb = "attach" if do_attach else "detach"
            raise RuntimeError(f"{verb} service unavailable: {srv_name}")

        req = req_type()
        req.model1_name = "six_arm"
        req.link1_name = "link6"
        req.model2_name = object_id
        req.link2_name = "link"

        resp_fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, resp_fut)
        resp = resp_fut.result()
        if not resp.success:
            verb = "attach" if do_attach else "detach"
            raise RuntimeError(f"{verb} {object_id}: {resp.message}")

    @staticmethod
    async def _svc_wait(client):
        return await client.wait_for_service(timeout_sec=10.0)

    @staticmethod
    async def _sleep(seconds):
        import asyncio
        await asyncio.sleep(seconds)

    # ==================================================================
    # Visualization & status
    # ==================================================================

    def _publish_marker(self, x, y, z, mid, label):
        # Cast to float — geometry_msgs.Point fields require float32,
        # which rejects bare ints (e.g. z=0) even in Python 3.
        x, y, z = float(x), float(y), float(z)
        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = self.MARKER_NS
        sphere.id = mid
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.scale.x = 0.1
        sphere.scale.y = 0.1
        sphere.scale.z = 0.1
        color_r = 0.0 if mid == self.MARKER_ID_PLACE() else 1.0
        color_g = 0.0 if mid == self.MARKER_ID_PLACE() else 1.0
        sphere.color.r = color_r
        sphere.color.g = color_g
        sphere.color.b = 0.0
        sphere.color.a = 0.7
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = z
        sphere.pose.orientation.w = 1.0
        sphere.lifetime.sec = 0
        self._marker_pub.publish(sphere)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = self.MARKER_NS
        text.id = mid + 1000
        text.type = Marker.TEXT_VIEW_FACING
        text.scale.z = 0.15
        text.color.a = 1.0
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + 0.25
        text.pose.orientation.w = 1.0
        text.text = label
        self._marker_pub.publish(text)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"status={status}")


# ───────────────────────────────────────────────────────────────────────
# CLI entry-point (standalone usage)
# ───────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = VisionGraspAdapter()

    node.get_logger().info("""
Available commands (call from Python or test via ros2 action):

  # In a second terminal after importing:
  >>> import asyncio
  >>> node.run_pick_at_target(dry_run=False)
  >>> node.run_place_at_zone(dry_run=False)
""")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
