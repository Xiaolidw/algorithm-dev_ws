#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sstream>
#include <iomanip>
#include <random>
#include <ctime>

namespace gazebo
{
    class AutoMoveObstaclePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            std::string timestamp = this->GetTimestamp();
            std::string model_name = _parent->GetName();
            this->log_prefix = "[" + timestamp + "][" + model_name + "] ";
            
            try
            {
                std::cout << this->log_prefix << "=====================================" << std::endl;
                std::cout << this->log_prefix << "AutoMoveObstaclePlugin loading..." << std::endl;
                
                this->model = _parent;
                
                std::cout << this->log_prefix << "Model name: " << this->model->GetName() << std::endl;
                std::cout << this->log_prefix << "World name: " << this->model->GetWorld()->Name() << std::endl;
                
                std::cout << this->log_prefix << "Reading parameters..." << std::endl;
                
                if (_sdf->HasElement("start_x"))
                {
                    this->start_x = _sdf->Get<double>("start_x");
                    std::cout << this->log_prefix << "start_x found in SDF: " << this->start_x << std::endl;
                }
                else
                {
                    this->start_x = -9.0;
                    std::cout << this->log_prefix << "start_x not found, using default: " << this->start_x << std::endl;
                }
                
                if (_sdf->HasElement("start_y"))
                {
                    this->start_y = _sdf->Get<double>("start_y");
                    std::cout << this->log_prefix << "start_y found in SDF: " << this->start_y << std::endl;
                }
                else
                {
                    this->start_y = -4.2;
                    std::cout << this->log_prefix << "start_y not found, using default: " << this->start_y << std::endl;
                }
                
                if (_sdf->HasElement("end_x"))
                {
                    this->end_x = _sdf->Get<double>("end_x");
                    std::cout << this->log_prefix << "end_x found in SDF: " << this->end_x << std::endl;
                }
                else
                {
                    this->end_x = -5.0;
                    std::cout << this->log_prefix << "end_x not found, using default: " << this->end_x << std::endl;
                }
                
                if (_sdf->HasElement("end_y"))
                {
                    this->end_y = _sdf->Get<double>("end_y");
                    std::cout << this->log_prefix << "end_y found in SDF: " << this->end_y << std::endl;
                }
                else
                {
                    this->end_y = -3.3;
                    std::cout << this->log_prefix << "end_y not found, using default: " << this->end_y << std::endl;
                }
                
                if (_sdf->HasElement("speed"))
                {
                    this->speed = _sdf->Get<double>("speed");
                    std::cout << this->log_prefix << "speed found in SDF: " << this->speed << std::endl;
                }
                else
                {
                    this->speed = 0.5;
                    std::cout << this->log_prefix << "speed not found, using default: " << this->speed << std::endl;
                }
                
                std::cout << this->log_prefix << "Parameter summary:" << std::endl;
                std::cout << this->log_prefix << "  Start position: (" << this->start_x << ", " << this->start_y << ")" << std::endl;
                std::cout << this->log_prefix << "  End position: (" << this->end_x << ", " << this->end_y << ")" << std::endl;
                std::cout << this->log_prefix << "  Speed: " << this->speed << " m/s" << std::endl;
                
                std::cout << this->log_prefix << "Calculating direction vector..." << std::endl;
                double dx = this->end_x - this->start_x;
                double dy = this->end_y - this->start_y;
                double distance = sqrt(dx*dx + dy*dy);
                
                std::cout << this->log_prefix << "  dx = end_x - start_x = " << this->end_x << " - " << this->start_x << " = " << dx << std::endl;
                std::cout << this->log_prefix << "  dy = end_y - start_y = " << this->end_y << " - " << this->start_y << " = " << dy << std::endl;
                std::cout << this->log_prefix << "  Total distance: " << distance << " m" << std::endl;
                
                if (distance > 0.001)
                {
                    this->dir_x = dx / distance;
                    this->dir_y = dy / distance;
                    std::cout << this->log_prefix << "  Direction vector: (" << this->dir_x << ", " << this->dir_y << ")" << std::endl;
                }
                else
                {
                    this->dir_x = 0;
                    this->dir_y = 0;
                    std::cout << this->log_prefix << "  WARNING: Start and end positions are the same!" << std::endl;
                    std::cout << this->log_prefix << "  Direction vector set to (0, 0)" << std::endl;
                }
                
                std::cout << this->log_prefix << "Setting initial position..." << std::endl;
                ignition::math::Pose3d pose(this->start_x, this->start_y, 0.375, 0, 0, 0);
                this->model->SetWorldPose(pose);
                std::cout << this->log_prefix << "  Initial position set to: (" << this->start_x << ", " << this->start_y << ")" << std::endl;
                
                this->current_x = this->start_x;
                this->current_y = this->start_y;
                std::cout << this->log_prefix << "  Current position initialized to: (" << this->current_x << ", " << this->current_y << ")" << std::endl;
                
                this->current_target_x = this->end_x;
                this->current_target_y = this->end_y;
                std::cout << this->log_prefix << "  Current target: (" << this->current_target_x << ", " << this->current_target_y << ")" << std::endl;
                
                std::cout << this->log_prefix << "Checking ROS status..." << std::endl;
                if (!rclcpp::ok())
                {
                    std::cout << this->log_prefix << "  ROS not initialized, initializing now..." << std::endl;
                    int argc = 0;
                    char** argv = nullptr;
                    rclcpp::init(argc, argv);
                }
                else
                {
                    std::cout << this->log_prefix << "  ROS already initialized" << std::endl;
                }
                
                std::string node_name = "auto_move_obstacle_" + model_name + "_" + this->GetRandomString(6);
                this->node = rclcpp::Node::make_shared(node_name);
                std::cout << this->log_prefix << "  ROS node created: " << node_name << std::endl;
                
                std::string topic_name = "/" + model_name + "/current_pose";
                this->pose_publisher = this->node->create_publisher<geometry_msgs::msg::Pose>(
                    topic_name, 10);
                std::cout << this->log_prefix << "  Position publisher created: " << topic_name << std::endl;
                
                std::cout << this->log_prefix << "Creating update connection..." << std::endl;
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&AutoMoveObstaclePlugin::OnUpdate, this));
                
                if (this->updateConnection)
                {
                    std::cout << this->log_prefix << "  Update connection created successfully" << std::endl;
                }
                else
                {
                    std::cerr << this->log_prefix << "  WARNING: Failed to create update connection!" << std::endl;
                }
                
                this->update_counter = 0;
                this->last_status_output = 0;
                this->last_ros_publish = 0;
                this->last_update_time = 0;
                this->direction_change_count = 0;
                
                std::cout << this->log_prefix << "Plugin loaded successfully!" << std::endl;
                std::cout << this->log_prefix << "=====================================" << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << this->log_prefix << "ERROR in Load function: " << e.what() << std::endl;
                throw;
            }
            catch (...)
            {
                std::cerr << this->log_prefix << "UNKNOWN ERROR in Load function" << std::endl;
                throw;
            }
        }
        
    public:
        void OnUpdate()
        {
            this->update_counter++;
            
            try
            {
                gazebo::common::Time current_time = this->model->GetWorld()->SimTime();
                double current_time_double = current_time.Double();
                double delta_time = current_time_double - this->last_update_time;
                
                // 修复：降低delta_time阈值
                if (delta_time < 0.001)
                {
                    return;
                }
                
                double move_distance = this->speed * delta_time;
                
                double old_x = this->current_x;
                double old_y = this->current_y;
                this->current_x += this->dir_x * move_distance;
                this->current_y += this->dir_y * move_distance;
                
                // 计算到当前目标的距离
                double distance_to_target = sqrt(
                    pow(this->current_target_x - this->current_x, 2) + 
                    pow(this->current_target_y - this->current_y, 2));
                
                // 改进的变向逻辑
                if (distance_to_target < 0.1)
                {
                    this->direction_change_count++;
                    std::cout << this->log_prefix << "=====================================" << std::endl;
                    std::cout << this->log_prefix << "DIRECTION CHANGE #" << this->direction_change_count << std::endl;
                    std::cout << this->log_prefix << "Reached target!" << std::endl;
                    std::cout << this->log_prefix << "  Position: (" << std::fixed << std::setprecision(3) << this->current_x << ", " << this->current_y << ")" << std::endl;
                    std::cout << this->log_prefix << "  Target: (" << this->current_target_x << ", " << this->current_target_y << ")" << std::endl;
                    
                    // 精确设置到目标位置
                    this->current_x = this->current_target_x;
                    this->current_y = this->current_target_y;
                    
                    // 切换目标
                    bool was_going_to_end = (this->current_target_x == this->end_x && 
                                           this->current_target_y == this->end_y);
                    
                    if (was_going_to_end)
                    {
                        this->current_target_x = this->start_x;
                        this->current_target_y = this->start_y;
                        std::cout << this->log_prefix << "  Switching to START position" << std::endl;
                    }
                    else
                    {
                        this->current_target_x = this->end_x;
                        this->current_target_y = this->end_y;
                        std::cout << this->log_prefix << "  Switching to END position" << std::endl;
                    }
                    
                    // 重新计算方向向量
                    double dx_new = this->current_target_x - this->current_x;
                    double dy_new = this->current_target_y - this->current_y;
                    double distance_new = sqrt(dx_new*dx_new + dy_new*dy_new);
                    
                    if (distance_new > 0.001)
                    {
                        this->dir_x = dx_new / distance_new;
                        this->dir_y = dy_new / distance_new;
                    }
                    else
                    {
                        this->dir_x = 0;
                        this->dir_y = 0;
                    }
                    
                    std::cout << this->log_prefix << "  New direction: (" << this->dir_x << ", " << this->dir_y << ")" << std::endl;
                    std::cout << this->log_prefix << "=====================================" << std::endl;
                    
                    // 立即更新模型位置
                    ignition::math::Pose3d new_pose(this->current_x, this->current_y, 0.375, 0, 0, 0);
                    this->model->SetWorldPose(new_pose);
                    
                    // 立即发布位置更新
                    this->PublishPosition();
                }
                else
                {
                    // 正常移动时更新位置
                    ignition::math::Pose3d new_pose(this->current_x, this->current_y, 0.375, 0, 0, 0);
                    this->model->SetWorldPose(new_pose);
                }
                
                // 更新最后更新时间
                this->last_update_time = current_time_double;
                
                // 控制ROS发布频率（约10Hz）
                if (current_time_double - this->last_ros_publish >= 0.1)
                {
                    this->PublishPosition();
                    this->last_ros_publish = current_time_double;
                }
                
                // 控制控制台输出频率（1秒一次）
                if (current_time_double - this->last_status_output >= 1.0)
                {
                    this->PrintStatus(current_time_double, distance_to_target);
                    this->last_status_output = current_time_double;
                }
                
                if (rclcpp::ok())
                {
                    rclcpp::spin_some(this->node);
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << this->log_prefix << "ERROR in OnUpdate function: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << this->log_prefix << "UNKNOWN ERROR in OnUpdate function" << std::endl;
            }
        }
        
    private:
        void PublishPosition()
        {
            if (this->pose_publisher && rclcpp::ok())
            {
                geometry_msgs::msg::Pose pose_msg;
                pose_msg.position.x = this->current_x;
                pose_msg.position.y = this->current_y;
                pose_msg.position.z = 0.375;
                pose_msg.orientation.w = 1.0;
                
                this->pose_publisher->publish(pose_msg);
            }
        }
        
        void PrintStatus(double current_time, double distance_to_target)
        {
            // 计算速度向量的大小
            double speed_magnitude = sqrt(this->dir_x*this->dir_x + this->dir_y*this->dir_y) * this->speed;
            
            std::cout << this->log_prefix 
                      << "Status | Time: " << std::fixed << std::setprecision(1) << current_time << "s"
                      << " | Position: (" << std::fixed << std::setprecision(3) << this->current_x << ", " << this->current_y << ")"
                      << " | Target: (" << this->current_target_x << ", " << this->current_target_y << ")"
                      << " | Distance: " << std::fixed << std::setprecision(2) << distance_to_target << "m"
                      << " | Speed: " << std::fixed << std::setprecision(2) << speed_magnitude << "m/s" << std::endl;
        }
        
        std::string GetTimestamp()
        {
            try
            {
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                
                #ifdef _WIN32
                struct tm buf;
                localtime_s(&buf, &in_time_t);
                ss << std::put_time(&buf, "%H:%M:%S");
                #else
                struct tm buf;
                localtime_r(&in_time_t, &buf);
                ss << std::put_time(&buf, "%H:%M:%S");
                #endif
                
                return ss.str();
            }
            catch (...)
            {
                static int fallback_counter = 0;
                std::stringstream ss;
                ss << "TIME" << std::setw(6) << std::setfill('0') << fallback_counter++;
                return ss.str().substr(0, 8);
            }
        }
        
        std::string GetRandomString(int length)
        {
            try
            {
                const std::string chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
                std::string result;
                
                std::random_device rd;
                std::mt19937 generator(rd() ^ static_cast<unsigned int>(std::time(nullptr)));
                std::uniform_int_distribution<> distribution(0, static_cast<int>(chars.size() - 1));
                
                for (int i = 0; i < length; ++i)
                {
                    result += chars[distribution(generator)];
                }
                return result;
            }
            catch (...)
            {
                static int fallback_counter = 0;
                std::stringstream ss;
                ss << "RAND" << std::setw(3) << std::setfill('0') << fallback_counter++;
                return ss.str().substr(0, length);
            }
        }
        
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        
        double start_x, start_y;
        double end_x, end_y;
        double speed;
        double dir_x, dir_y;
        double current_target_x, current_target_y;
        double current_x, current_y;
        
        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
        
        std::string log_prefix;
        int update_counter;
        double last_status_output;
        double last_ros_publish;
        double last_update_time;
        int direction_change_count;
    };
    
    GZ_REGISTER_MODEL_PLUGIN(AutoMoveObstaclePlugin)
}