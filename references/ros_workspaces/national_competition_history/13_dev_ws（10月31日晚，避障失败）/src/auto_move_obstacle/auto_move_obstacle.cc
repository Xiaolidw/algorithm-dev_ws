#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class AutoMoveObstacle : public ModelPlugin
{
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    double speed = 0.05; // 速度改为0.1

    // 路径参数
    double startX, startY;       // 起点
    double midX, midY;           // 中间点（可选）
    double endX, endY;           // 终点
    int pathStage = 0;           // 路径阶段（0:起点→中间点；1:中间点→终点；2:终点→中间点；3:中间点→起点）
    bool hasMidPoint = false;    // 是否有中间点

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model = _model;
        if (!model)
        {
            gzerr << "Model pointer is null!\n";
            return;
        }

        // 绑定更新事件
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AutoMoveObstacle::OnUpdate, this));

        // 读取路径参数
        startX = _sdf->Get<double>("start_x");
        startY = _sdf->Get<double>("start_y");
        endX = _sdf->Get<double>("end_x");
        endY = _sdf->Get<double>("end_y");

        // 检查是否有中间点
        if (_sdf->HasElement("mid_x") && _sdf->HasElement("mid_y"))
        {
            midX = _sdf->Get<double>("mid_x");
            midY = _sdf->Get<double>("mid_y");
            hasMidPoint = true;
        }

        // 初始化位置到起点
        model->SetWorldPose(ignition::math::Pose3d(startX, startY, 0.5, 0, 0, 0));
    }

    void OnUpdate()
    {
        ignition::math::Pose3d currentPose = model->WorldPose();
        double currentX = currentPose.Pos().X();
        double currentY = currentPose.Pos().Y();

        // 根据阶段确定当前目标点（核心：实现往返）
        double targetX, targetY;
        if (hasMidPoint)
        {
            // 多段路径往返（起点→中间点→终点→中间点→起点）
            switch(pathStage)
            {
                case 0: targetX = midX; targetY = midY; break;   // 起点→中间点
                case 1: targetX = endX; targetY = endY; break;   // 中间点→终点
                case 2: targetX = midX; targetY = midY; break;   // 终点→中间点
                case 3: targetX = startX; targetY = startY; break; // 中间点→起点
                default: pathStage = 0; targetX = midX; targetY = midY;
            }
        }
        else
        {
            // 单段路径往返（起点→终点→起点）
            if (pathStage == 0)
            {
                targetX = endX; targetY = endY;
            }
            else
            {
                targetX = startX; targetY = startY;
            }
        }

        // 计算到目标点的距离和方向
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = sqrt(dx*dx + dy*dy);

        // 到达目标点时切换阶段（循环往返）
        if (distance < 0.1)
        {
            if (hasMidPoint)
            {
                pathStage = (pathStage + 1) % 4; // 多段路径循环（0→1→2→3→0）
            }
            else
            {
                pathStage = (pathStage + 1) % 2; // 单段路径循环（0→1→0）
            }
            return;
        }

        // 计算新位置（速度0.1）
        double dirX = dx / distance;
        double dirY = dy / distance;
        double delta = speed * 0.01; // 步长 = 速度 * 仿真时间间隔（0.01秒）
        double newX = currentX + dirX * delta;
        double newY = currentY + dirY * delta;

        // 更新位置
        model->SetWorldPose(ignition::math::Pose3d(newX, newY, 0.5, 0, 0, 0));
    }
};

GZ_REGISTER_MODEL_PLUGIN(AutoMoveObstacle)
}
