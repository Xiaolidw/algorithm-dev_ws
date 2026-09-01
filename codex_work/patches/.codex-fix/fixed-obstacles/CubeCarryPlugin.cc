// 载运插件: 在物理循环内把被"夹持"的方块刚性绑定到机械臂工具端。
//
// 背景: LinkAttacher 的跨模型焊接关节与接触求解互相注入冲量, 会把轻量
// 底盘弹飞; 而基于 SetEntityState 服务调用的跟随(30/100Hz)要么抖动要么
// 在负载下饿死。本插件在 OnUpdate(物理步)内直接用 Gazebo API 读取
// six_arm::link6 的世界位姿并 SetWorldPose 方块——零 ROS 流量、零延迟、
// 零重力坠落, 视觉与物理上完全刚性。
//
// 接口(std_msgs/String, 队列1):
//   /manipulation/carry_set  "carry:<cube_model>"  开始载运
//   /manipulation/carry_set  "release"             结束载运(方块留在原地)
//
// 工具模式在接到命令时捕获方块相对 link6 的真实位姿，因此不会发生
// “从远处吸到夹爪中心”的瞬移；后续只保持这个已验证的接触变换。

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <functional>
#include <cmath>

namespace moon_warehouse
{

class CubeCarryPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world,
            sdf::ElementPtr sdf) override
  {
    world_ = world;
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("cube_carry_plugin");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    carry_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/manipulation/carry_status", qos);

    carry_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/manipulation/carry_set", qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const std::string & data = msg->data;
        if (data == "release") {
          if (!carried_.empty()) {
            auto cube = world_->ModelByName(carried_);
            if (cube) {
              (void) cube;
            }
          }
          carried_.clear();
          mode_ = CarryMode::kNone;
          PublishStatus("none");
          gzmsg << "[cube_carry] released\n";
          return;
        }
        if (data.rfind("carry:", 0) == 0 || data.rfind("tool:", 0) == 0) {
          StartCarry(data.substr(data.find(':') + 1), CarryMode::kTool);
          return;
        }
        if (data.rfind("base:", 0) == 0) {
          StartCarry(data.substr(5), CarryMode::kBase);
        }
      });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CubeCarryPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[cube_carry] plugin loaded\n";
  }

  void OnUpdate(const gazebo::common::UpdateInfo &)
  {
    // A WorldPlugin owns no executor.  Without spin_some, carry commands are
    // received at DDS level but their callback never runs.
    rclcpp::spin_some(node_);
    auto robot = world_->ModelByName("six_arm");
    if (!robot) {
      return;
    }
    StabilizePlanarBase(robot);
    if (carried_.empty()) {
      return;
    }
    auto cube = world_->ModelByName(carried_);
    if (!cube) {
      carried_.clear();
      return;
    }

    ignition::math::Pose3d cube_pose;
    if (mode_ == CarryMode::kTool) {
      auto tool = robot->GetLink("link6");
      if (!tool) {
        return;
      }
      const ignition::math::Pose3d tool_pose = tool->WorldPose();
      cube_pose.Pos() = tool_pose.Pos() +
        tool_pose.Rot().RotateVector(tool_relative_position_);
      cube_pose.Rot() = tool_pose.Rot() * tool_relative_rotation_;
    } else if (mode_ == CarryMode::kBase) {
      const ignition::math::Pose3d base_pose = robot->WorldPose();
      cube_pose.Pos() = base_pose.Pos() +
        base_pose.Rot().RotateVector(base_relative_position_);
      cube_pose.Rot() = base_pose.Rot() * base_relative_rotation_;
    } else {
      return;
    }

    // Physics-step pose update: no ROS service latency, gravity sag or
    // contact impulse.  During base mode the cube changes only with chassis
    // translation/yaw, not with vertical arm-servo motion.
    cube->SetWorldPose(cube_pose);
    cube->SetLinearVel(ignition::math::Vector3d::Zero);
    cube->SetAngularVel(ignition::math::Vector3d::Zero);
  }

private:
  enum class CarryMode {kNone, kTool, kBase};

  void StabilizePlanarBase(const gazebo::physics::ModelPtr & robot)
  {
    // The competition robot is a planar mobile base.  Real wheel odometry
    // controls x/y/yaw; roll, pitch and vertical motion are non-commandable
    // failure modes caused by contact impulses.  Constraining only those
    // three degrees of freedom prevents a wall/cube collision from launching
    // the whole robot while preserving genuine planar collision avoidance.
    ignition::math::Pose3d pose = robot->WorldPose();
    if (!planar_height_initialized_) {
      planar_height_ = pose.Pos().Z();
      planar_height_initialized_ = true;
      gzmsg << "[cube_carry] planar base lock z=" << planar_height_ << "\n";
    }
    ignition::math::Vector3d linear = robot->WorldLinearVel();
    ignition::math::Vector3d angular = robot->WorldAngularVel();
    const bool pose_out_of_plane =
      std::abs(pose.Pos().Z() - planar_height_) > 0.05 ||
      std::abs(pose.Rot().Roll()) > 0.15 ||
      std::abs(pose.Rot().Pitch()) > 0.15;
    // 正常平地运动时绝不调用 SetWorldPose/Set*Vel。差速轮的转向力矩由
    // 物理引擎自然积分；旧实现每个周期重写角速度，会把 yaw 旋转冻结。
    // 仅当碰撞确实产生非平面越界时介入，并完整保留 x/y/yaw 分量。
    if (!pose_out_of_plane) {
      return;
    }
    pose.Pos().Z(planar_height_);
    pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, pose.Rot().Yaw());
    robot->SetWorldPose(pose);
    linear.Z(0.0);
    robot->SetLinearVel(linear);
    angular.X(0.0);
    angular.Y(0.0);
    robot->SetAngularVel(angular);
  }

  void StartCarry(const std::string & cube_name, CarryMode mode)
  {
    auto cube = world_->ModelByName(cube_name);
    auto robot = world_->ModelByName("six_arm");
    if (!cube || !robot) {
      gzwarn << "[cube_carry] unknown cube or robot: " << cube_name << "\n";
      return;
    }
    carried_ = cube_name;
    mode_ = mode;
    // Keep normal Gazebo dynamics. The physical-loop SetWorldPose below is
    // the only carrier; altering the model's static/collision/gravity flags
    // prevents this Gazebo version from applying the update correctly.
    cube->SetLinearVel(ignition::math::Vector3d::Zero);
    cube->SetAngularVel(ignition::math::Vector3d::Zero);
    if (mode == CarryMode::kBase) {
      const ignition::math::Pose3d base_pose = robot->WorldPose();
      const ignition::math::Pose3d cube_pose = cube->WorldPose();
      base_relative_position_ = base_pose.Rot().RotateVectorReverse(
        cube_pose.Pos() - base_pose.Pos());
      base_relative_rotation_ = base_pose.Rot().Inverse() * cube_pose.Rot();
    } else if (mode == CarryMode::kTool) {
      auto tool = robot->GetLink("link6");
      if (!tool) {
        carried_.clear();
        mode_ = CarryMode::kNone;
        return;
      }
      const ignition::math::Pose3d tool_pose = tool->WorldPose();
      const ignition::math::Pose3d cube_pose = cube->WorldPose();
      tool_relative_position_ = tool_pose.Rot().RotateVectorReverse(
        cube_pose.Pos() - tool_pose.Pos());
      tool_relative_rotation_ = tool_pose.Rot().Inverse() * cube_pose.Rot();
    }
    gzmsg << "[cube_carry] " << (mode == CarryMode::kBase ? "base" : "tool")
          << " carry " << cube_name << "\n";
    PublishStatus(std::string(mode == CarryMode::kBase ? "base:" : "tool:")
      + cube_name);
  }

  void PublishStatus(const std::string & status)
  {
    if (!carry_status_pub_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = status;
    carry_status_pub_->publish(message);
  }

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr carry_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr carry_status_pub_;
  std::string carried_;
  CarryMode mode_{CarryMode::kNone};
  ignition::math::Vector3d tool_relative_position_;
  ignition::math::Quaterniond tool_relative_rotation_;
  ignition::math::Vector3d base_relative_position_;
  ignition::math::Quaterniond base_relative_rotation_;
  bool planar_height_initialized_{false};
  double planar_height_{0.0};
};

GZ_REGISTER_WORLD_PLUGIN(CubeCarryPlugin)

}  // namespace moon_warehouse
