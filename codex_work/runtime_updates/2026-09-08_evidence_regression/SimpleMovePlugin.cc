#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace gazebo
{
    class SimpleMovePlugin : public ModelPlugin
    {
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        physics::LinkPtr link;
        
        // 运动参数
        double speed;
        double force;
        double start_x;
        double start_y;
        double end_x;
        double end_y;
        bool moving_to_end;
        
        std::string log_prefix;
        
        // ROS 2相关
        std::shared_ptr<rclcpp::Node> ros_node;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
        std::string pose_topic;
        double pose_publish_rate;
        double last_pose_publish_time;
        double motion_z;
        double last_control_time;
        double path_length;
        double path_progress;
        ignition::math::Quaterniond locked_rotation;
        
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->link = this->model->GetLink("link");
            this->motion_z = this->model->WorldPose().Pos().Z();
            this->last_control_time = -1.0;
            this->last_pose_publish_time = -1.0;
            
            std::string model_name = _parent->GetName();
            this->log_prefix = "[" + model_name + "] ";
            
            std::cout << this->log_prefix << "SimpleMovePlugin loading..." << std::endl;
            
            // 检查link是否有效
            if (!this->link)
            {
                std::cerr << this->log_prefix << "ERROR: Failed to get link 'link'!" << std::endl;
                return;
            }
            
            // 读取参数
            this->speed = _sdf->Get<double>("speed", 0.5).first;
            this->force = _sdf->Get<double>("force", 10.0).first;
            this->pose_publish_rate =
                _sdf->Get<double>("pose_publish_rate", 10.0).first;
            if (this->pose_publish_rate <= 0.0)
            {
                this->pose_publish_rate = 10.0;
            }
            // 读取起点和终点
            this->start_x = _sdf->Get<double>("start_x", 0.0).first;
            this->start_y = _sdf->Get<double>("start_y", 0.0).first;
            this->end_x = _sdf->Get<double>("end_x", 5.0).first;
            this->end_y = _sdf->Get<double>("end_y", 0.0).first;
            this->path_length = std::hypot(
                this->end_x - this->start_x,
                this->end_y - this->start_y);
            this->path_progress = 0.0;
            this->locked_rotation = this->model->WorldPose().Rot();
            
            this->moving_to_end = true;
            
            // 输出参数信息
            std::cout << this->log_prefix << "Parameters:" << std::endl;
            std::cout << this->log_prefix << "  Speed: " << this->speed << " m/s" << std::endl;
            std::cout << this->log_prefix << "  Force: " << this->force << " N" << std::endl;
            std::cout << this->log_prefix << "  Pose publish rate: "
                      << this->pose_publish_rate << " Hz" << std::endl;
            std::cout << this->log_prefix << "  Path: (" << this->start_x << "," << this->start_y 
                      << ") -> (" << this->end_x << "," << this->end_y << ")" << std::endl;
            
            // The obstacle is a prescribed competition hazard, not a yielding
            // dynamic body.  Kinematic motion plus an internal path coordinate
            // prevents robot contact from changing its position, height or yaw.
            this->link->SetLinearDamping(0.0);
            this->link->SetAngularDamping(0.0);
            this->link->SetKinematic(true);
            this->link->SetGravityMode(false);
            this->model->SetAutoDisable(false);
            
            // 初始化ROS 2
            this->InitROS(model_name);
            
            // 创建更新连接
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&SimpleMovePlugin::OnUpdate, this, _1));
            
            std::cout << this->log_prefix << "Plugin loaded successfully!" << std::endl;
        }
        
        void InitROS(const std::string& model_name)
        {
            // 初始化ROS 2节点和发布器
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }
            
            // 创建节点
            this->ros_node = std::make_shared<rclcpp::Node>("simple_move_plugin_" + model_name);
            
            // 创建位置发布器
            this->pose_topic = "/" + model_name + "/current_pose";
            this->pose_publisher = this->ros_node->create_publisher<geometry_msgs::msg::Pose>(
                this->pose_topic, 10);
            
            RCLCPP_INFO(this->ros_node->get_logger(), "Publishing pose to: %s", this->pose_topic.c_str());
        }
        
        void OnUpdate(const common::UpdateInfo &_info)
        {
            try
            {
                double current_time = _info.simTime.Double();
                double control_dt = this->last_control_time < 0.0
                    ? 0.0
                    : current_time - this->last_control_time;
                this->last_control_time = current_time;
                control_dt = std::max(0.0, std::min(control_dt, 0.05));
                if (this->path_length <= 1e-9)
                {
                    return;
                }

                const double direction = this->moving_to_end ? 1.0 : -1.0;
                this->path_progress += direction * this->speed * control_dt;
                if (this->path_progress >= this->path_length)
                {
                    this->path_progress = this->path_length;
                    this->SwitchDirection();
                }
                else if (this->path_progress <= 0.0)
                {
                    this->path_progress = 0.0;
                    this->SwitchDirection();
                }

                const double unit_x = (this->end_x - this->start_x) / this->path_length;
                const double unit_y = (this->end_y - this->start_y) / this->path_length;
                ignition::math::Pose3d commanded_pose(
                    this->start_x + unit_x * this->path_progress,
                    this->start_y + unit_y * this->path_progress,
                    this->motion_z,
                    this->locked_rotation.Roll(),
                    this->locked_rotation.Pitch(),
                    this->locked_rotation.Yaw());
                this->model->SetWorldPose(commanded_pose, true, true);

                const double commanded_direction = this->moving_to_end ? 1.0 : -1.0;
                ignition::math::Vector3d commanded_velocity(
                    unit_x * this->speed * commanded_direction,
                    unit_y * this->speed * commanded_direction,
                    0.0);
                this->link->SetLinearVel(commanded_velocity);
                this->link->SetAngularVel(ignition::math::Vector3d::Zero);

                // Keep prediction input poses at a stable configurable rate.
                const double pose_publish_period = 1.0 / this->pose_publish_rate;
                if (this->last_pose_publish_time < 0.0 ||
                    current_time < this->last_pose_publish_time ||
                    current_time - this->last_pose_publish_time >=
                        pose_publish_period - 1e-9)
                {
                    this->PublishPose(commanded_pose);
                    this->last_pose_publish_time = current_time;
                }
                
                // 定期输出状态
                static double last_print_time = 0;
                if (current_time - last_print_time >= 5.0)
                {
                    const double remaining = this->moving_to_end
                        ? this->path_length - this->path_progress
                        : this->path_progress;
                    this->PrintStatus(
                        current_time, commanded_pose, commanded_velocity, remaining);
                    last_print_time = current_time;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << this->log_prefix << "ERROR in OnUpdate: " << e.what() << std::endl;
            }
        }
        
        void PublishPose(const ignition::math::Pose3d& pose)
        {
            // 发布当前位置信息
            if (this->pose_publisher->get_subscription_count() > 0 && rclcpp::ok())
            {
                geometry_msgs::msg::Pose ros_pose;
                ros_pose.position.x = pose.Pos().X();
                ros_pose.position.y = pose.Pos().Y();
                ros_pose.position.z = pose.Pos().Z();
                
                ros_pose.orientation.x = pose.Rot().X();
                ros_pose.orientation.y = pose.Rot().Y();
                ros_pose.orientation.z = pose.Rot().Z();
                ros_pose.orientation.w = pose.Rot().W();
                
                this->pose_publisher->publish(ros_pose);
                
                // 处理ROS回调
                rclcpp::spin_some(this->ros_node);
            }
        }
        
        void SwitchDirection()
        {
            this->moving_to_end = !this->moving_to_end;
            std::string new_target = this->moving_to_end ? "END" : "START";
            std::cout << this->log_prefix << "----------------------------------------" << std::endl;
            std::cout << this->log_prefix << "Reached target, now moving to " << new_target << std::endl;
        }
        
        void PrintStatus(double current_time, const ignition::math::Pose3d& pose,
                        const ignition::math::Vector3d& linear_vel, double distance)
        {
            std::string target_str = this->moving_to_end ? "END" : "START";
            
            std::cout << this->log_prefix 
                      << "Time: " << current_time << "s | "
                      << "Pos: (" << pose.Pos().X() << "," << pose.Pos().Y() << ") | "
                      << "Vel: " << linear_vel.Length() << "m/s | "
                      << "To" << target_str << ": " << distance << "m" << std::endl;
        }
        
        ~SimpleMovePlugin()
        {
            this->updateConnection.reset();
            this->pose_publisher.reset();
            this->ros_node.reset();
        }
    };
    
    GZ_REGISTER_MODEL_PLUGIN(SimpleMovePlugin)
}
