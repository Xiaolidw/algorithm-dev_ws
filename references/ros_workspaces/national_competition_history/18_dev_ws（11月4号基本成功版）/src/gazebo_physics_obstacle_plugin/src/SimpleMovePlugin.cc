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
        
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->link = this->model->GetLink("link");
            
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
            
            // 读取起点和终点
            this->start_x = _sdf->Get<double>("start_x", 0.0).first;
            this->start_y = _sdf->Get<double>("start_y", 0.0).first;
            this->end_x = _sdf->Get<double>("end_x", 5.0).first;
            this->end_y = _sdf->Get<double>("end_y", 0.0).first;
            
            this->moving_to_end = true;
            
            // 输出参数信息
            std::cout << this->log_prefix << "Parameters:" << std::endl;
            std::cout << this->log_prefix << "  Speed: " << this->speed << " m/s" << std::endl;
            std::cout << this->log_prefix << "  Force: " << this->force << " N" << std::endl;
            std::cout << this->log_prefix << "  Path: (" << this->start_x << "," << this->start_y 
                      << ") -> (" << this->end_x << "," << this->end_y << ")" << std::endl;
            
            // 设置阻尼
            this->link->SetLinearDamping(0.2);
            this->link->SetAngularDamping(2.0);
            
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
                // 获取当前状态
                ignition::math::Pose3d pose = this->model->WorldPose();
                ignition::math::Vector3d linear_vel = this->link->WorldLinearVel();
                
                double current_x = pose.Pos().X();
                double current_y = pose.Pos().Y();
                
                // 发布位置信息
                this->PublishPose(pose);
                
                // 确定目标点
                double target_x = this->moving_to_end ? this->end_x : this->start_x;
                double target_y = this->moving_to_end ? this->end_y : this->start_y;
                
                // 计算到目标的距离和方向
                double dx = target_x - current_x;
                double dy = target_y - current_y;
                double distance = sqrt(dx*dx + dy*dy);
                
                // 检查是否到达目标
                if (distance < 0.2)
                {
                    this->SwitchDirection();
                    target_x = this->moving_to_end ? this->end_x : this->start_x;
                    target_y = this->moving_to_end ? this->end_y : this->start_y;
                    dx = target_x - current_x;
                    dy = target_y - current_y;
                    distance = sqrt(dx*dx + dy*dy);
                }
                
                // 如果距离太小，不施加力
                if (distance < 0.05)
                {
                    this->PrintStatus(_info.simTime.Double(), pose, linear_vel, 0.0);
                    return;
                }
                
                // 计算方向
                double dir_x = dx / distance;
                double dir_y = dy / distance;
                
                // 计算当前速度在目标方向上的分量
                double vel_in_dir = linear_vel.X() * dir_x + linear_vel.Y() * dir_y;
                
                // 如果速度小于目标速度，施加力
                if (vel_in_dir < this->speed)
                {
                    ignition::math::Vector3d force_vec(dir_x * this->force, 
                                                     dir_y * this->force, 
                                                     0.0);
                    
                    // 在质心位置施加力
                    ignition::math::Vector3d center_of_mass = this->link->GetInertial()->Pose().Pos();
                    this->link->AddForceAtRelativePosition(force_vec, center_of_mass);
                }
                
                // 定期输出状态
                static double last_print_time = 0;
                double current_time = _info.simTime.Double();
                
                if (current_time - last_print_time >= 1.0)
                {
                    this->PrintStatus(current_time, pose, linear_vel, distance);
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
            // 析构函数
            if (rclcpp::ok())
            {
                rclcpp::shutdown();
            }
        }
    };
    
    GZ_REGISTER_MODEL_PLUGIN(SimpleMovePlugin)
}