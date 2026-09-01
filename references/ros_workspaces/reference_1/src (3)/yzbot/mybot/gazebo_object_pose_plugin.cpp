#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "mybot/msg/object_pose.hpp"
#include "mybot/msg/object_pose_array.hpp"

namespace gazebo
{
  class ObjectPosePlugin : public WorldPlugin
  {
  private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<mybot::msg::ObjectPoseArray>::SharedPtr pose_pub_;
    event::ConnectionPtr update_conn_;
    physics::WorldPtr world_;
    int frame_count_ = 0;

  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override
    {
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }
      ros_node_ = rclcpp::Node::make_shared("gazebo_object_pose_node");

      pose_pub_ = ros_node_->create_publisher<mybot::msg::ObjectPoseArray>(
          "/object_detection/object_poses", 10);

      RCLCPP_INFO(ros_node_->get_logger(), " WorldPlugin 位姿插件已加载！（含机器人坐标发布）");

      world_ = _world;
      update_conn_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ObjectPosePlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      frame_count_++;
      mybot::msg::ObjectPoseArray pose_array_msg;

      pose_array_msg.header.stamp = ros_node_->get_clock()->now();
      pose_array_msg.header.frame_id = "world";  //  使用 world 系

      auto all_models = world_->Models();
      for (auto model : all_models)
      {
        std::string model_name = model->GetName();
        mybot::msg::ObjectPose obj_msg;
        obj_msg.id = model_name;

        //  检测类型（包括机器人 + 动态障碍物）
        if (model_name.find("red_cube") != std::string::npos)
          obj_msg.type = "block_red";
        else if (model_name.find("blue_cube") != std::string::npos)
          obj_msg.type = "block_blue";
        else if (model_name.find("zone_") != std::string::npos)
          obj_msg.type = "zone";
        else if (model_name == "six_arm")
          obj_msg.type = "robot";  // 机器人本身
        else if (model_name.find("obstacle") != std::string::npos)
          obj_msg.type = "obstacle_dynamic";  // 新增：obstacle2 / obstacle3 等动态障碍物
        else
          continue;

        //获取坐标
        ignition::math::Pose3d gazebo_pose = model->WorldPose();

        obj_msg.pose.position.x = gazebo_pose.Pos().X();
        obj_msg.pose.position.y = gazebo_pose.Pos().Y();
        obj_msg.pose.position.z = gazebo_pose.Pos().Z();
        obj_msg.pose.orientation.x = gazebo_pose.Rot().X();
        obj_msg.pose.orientation.y = gazebo_pose.Rot().Y();
        obj_msg.pose.orientation.z = gazebo_pose.Rot().Z();
        obj_msg.pose.orientation.w = gazebo_pose.Rot().W();

        pose_array_msg.objects.push_back(obj_msg);
      }

      // 每 10 帧发布一次
      if (frame_count_ % 10 == 0)
      {
        pose_pub_->publish(pose_array_msg);

        if (frame_count_ % 100 == 0)
        {
          RCLCPP_INFO(ros_node_->get_logger(),
                      "检测到：红方块%d个 | 蓝方块%d个 | 任务点%d个 | 机器人%d个",
                      count_object_type(pose_array_msg, "block_red"),
                      count_object_type(pose_array_msg, "block_blue"),
                      count_object_type(pose_array_msg, "zone"),
                      count_object_type(pose_array_msg, "robot"));
          // 🔹 这里暂时没输出 obstacle_dynamic 的数量，应你要求不改其他逻辑
        }
      }
    }

    int count_object_type(const mybot::msg::ObjectPoseArray &msg, const std::string &type)
    {
      int count = 0;
      for (const auto &obj : msg.objects)
      {
        if (obj.type == type)
          count++;
      }
      return count;
    }

    void Fini()
    {
      rclcpp::shutdown();
      RCLCPP_INFO(ros_node_->get_logger(), "位姿插件已关闭！");
    }
  };

  GZ_REGISTER_WORLD_PLUGIN(ObjectPosePlugin)
}

