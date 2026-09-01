#ifndef MOVING_OBSTACLE_PLUGIN_HH
#define MOVING_OBSTACLE_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class MovingObstaclePlugin : public ModelPlugin
  {
    public: MovingObstaclePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // 检查模型指针是否有效
      if (!_model)
      {
        gzerr << "No model pointer provided!\n";
        return;
      }

      // 保存模型指针
      this->model = _model;

      // 获取参数
      this->speed = _sdf->Get<double>("speed", 0.2).first;
      this->amplitude = _sdf->Get<double>("amplitude", 2.0).first;
      this->updateRate = _sdf->Get<double>("update_rate", 50.0).first;

      gzmsg << "MovingObstaclePlugin loaded:\n";
      gzmsg << "  Speed: " << this->speed << " m/s\n";
      gzmsg << "  Amplitude: " << this->amplitude << " m\n";
      gzmsg << "  Update rate: " << this->updateRate << " Hz\n";

      // 初始化位置
      this->startPose = this->model->WorldPose();
      this->currentTime = 0.0;

      // 创建更新定时器
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovingObstaclePlugin::OnUpdate, this, std::placeholders::_1));
    }

    public: void OnUpdate(const common::UpdateInfo &_info)
    {
      // 计算时间增量
      double dt = _info.simTime.Double() - this->currentTime;
      this->currentTime = _info.simTime.Double();

      // 计算新位置（正弦运动）
      double x = this->startPose.Pos().X() + 
                 this->amplitude * sin(this->speed * this->currentTime);
      double y = this->startPose.Pos().Y();
      double z = this->startPose.Pos().Z();

      // 设置新位置
      ignition::math::Pose3d newPose(x, y, z, 0, 0, 0);
      this->model->SetWorldPose(newPose);
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ignition::math::Pose3d startPose;
    private: double speed;
    private: double amplitude;
    private: double updateRate;
    private: double currentTime;
  };

  GZ_REGISTER_MODEL_PLUGIN(MovingObstaclePlugin)
}
#endif
