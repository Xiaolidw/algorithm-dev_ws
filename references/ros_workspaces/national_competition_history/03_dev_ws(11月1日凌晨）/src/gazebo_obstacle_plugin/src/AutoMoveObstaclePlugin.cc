#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <chrono>
#include <thread>

namespace gazebo
{
    class AutoMoveObstaclePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // 存储模型指针
            this->model = _parent;
            
            // 获取参数
            this->start_x = _sdf->Get<double>("start_x", -9.0).first;
            this->start_y = _sdf->Get<double>("start_y", -4.2).first;
            this->end_x = _sdf->Get<double>("end_x", -5.0).first;
            this->end_y = _sdf->Get<double>("end_y", -3.3).first;
            this->speed = _sdf->Get<double>("speed", 0.05).first;
            
            std::cout << "AutoMoveObstaclePlugin loaded for model: " << this->model->GetName() << std::endl;
            std::cout << "Start position: (" << this->start_x << ", " << this->start_y << ")" << std::endl;
            std::cout << "End position: (" << this->end_x << ", " << this->end_y << ")" << std::endl;
            std::cout << "Speed: " << this->speed << " m/s" << std::endl;
            
            // 计算方向向量
            double dx = this->end_x - this->start_x;
            double dy = this->end_y - this->start_y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance > 0)
            {
                this->dir_x = dx / distance;
                this->dir_y = dy / distance;
            }
            else
            {
                this->dir_x = 0;
                this->dir_y = 0;
            }
            
            // 设置初始位置
            ignition::math::Pose3d pose(this->start_x, this->start_y, 0.375, 0, 0, 0);
            this->model->SetWorldPose(pose);
            
            // 创建更新连接
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&AutoMoveObstaclePlugin::OnUpdate, this));
                
            // 记录当前目标
            this->current_target_x = this->end_x;
            this->current_target_y = this->end_y;
        }
        
    public:
        void OnUpdate()
        {
            // 获取当前位置
            ignition::math::Pose3d current_pose = this->model->WorldPose();
            double x = current_pose.Pos().X();
            double y = current_pose.Pos().Y();
            
            // 计算到目标的距离
            double distance_to_target = sqrt(
                pow(this->current_target_x - x, 2) + 
                pow(this->current_target_y - y, 2));
            
            // 如果到达目标，切换方向
            if (distance_to_target < 0.1)
            {
                std::cout << "Switching direction for " << this->model->GetName() << std::endl;
                
                if (this->current_target_x == this->end_x)
                {
                    this->current_target_x = this->start_x;
                    this->current_target_y = this->start_y;
                    this->dir_x = -this->dir_x;
                    this->dir_y = -this->dir_y;
                }
                else
                {
                    this->current_target_x = this->end_x;
                    this->current_target_y = this->end_y;
                    this->dir_x = -this->dir_x;
                    this->dir_y = -this->dir_y;
                }
            }
            
            // 计算移动距离（假设更新频率为100Hz）
            double move_distance = this->speed / 100.0;
            
            // 计算新位置
            double new_x = x + this->dir_x * move_distance;
            double new_y = y + this->dir_y * move_distance;
            
            // 设置新位置
            ignition::math::Pose3d new_pose(new_x, new_y, 0.375, 0, 0, 0);
            this->model->SetWorldPose(new_pose);
        }
        
    private:
        // 模型指针
        physics::ModelPtr model;
        
        // 更新连接
        event::ConnectionPtr updateConnection;
        
        // 参数
        double start_x, start_y;
        double end_x, end_y;
        double speed;
        double dir_x, dir_y;
        double current_target_x, current_target_y;
    };
    
    // 注册插件
    GZ_REGISTER_MODEL_PLUGIN(AutoMoveObstaclePlugin)
}
