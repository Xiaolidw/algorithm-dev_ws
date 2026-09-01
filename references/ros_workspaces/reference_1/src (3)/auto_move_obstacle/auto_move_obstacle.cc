#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

namespace gazebo
{
class AutoMoveObstacle : public ModelPlugin
{
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    // 运动相关参数
    double speed = 0.02;  // 运动线速度（m/s 的近似），越大越快
    double startX = 0.0, startY = 0.0;
    double endX   = 1.0, endY   = 0.0;

    // 归一化路径进度 [0,1] ，0 在起点，1 在终点
    double sParam = 0.0;
    int direction = 1; // 1=起点->终点，-1=终点->起点

    // 路径长度
    double pathLen = 1.0;

    // 是否启用 S 型轨迹
    bool useSCurve = false;
    double sAmplitude = 0.3; // 左右摆动幅度（米）
    int sWaves = 1;          // 从起点到终点有几个“S”

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        model = _model;
        if (!model)
        {
            gzerr << "[AutoMoveObstacle] Model pointer is null!\n";
            return;
        }

        // 基本参数读取（和你原来的兼容）
        if (_sdf->HasElement("speed"))
            speed = _sdf->Get<double>("speed");

        startX = _sdf->Get<double>("start_x", 0.0).first;
        startY = _sdf->Get<double>("start_y", 0.0).first;
        endX   = _sdf->Get<double>("end_x", 1.0).first;
        endY   = _sdf->Get<double>("end_y", 0.0).first;

        // S 型相关参数
        if (_sdf->HasElement("s_curve"))
            useSCurve = _sdf->Get<bool>("s_curve");

        if (_sdf->HasElement("s_amplitude"))
            sAmplitude = _sdf->Get<double>("s_amplitude");

        if (_sdf->HasElement("s_waves"))
            sWaves = _sdf->Get<int>("s_waves");

        // 计算路径长度
        pathLen = std::hypot(endX - startX, endY - startY);
        if (pathLen < 1e-6)
        {
            gzerr << "[AutoMoveObstacle] Path length too small, use default 1.0\n";
            pathLen = 1.0;
        }

        // 初始放在起点
        sParam = 0.0;
        direction = 1;
        model->SetWorldPose(ignition::math::Pose3d(startX, startY, 0.3, 0, 0, 0));

        // 注册更新函数
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AutoMoveObstacle::OnUpdate, this));

        gzmsg << "[AutoMoveObstacle] Loaded. speed=" << speed
              << " start=(" << startX << "," << startY << ")"
              << " end=(" << endX << "," << endY << ")"
              << " useSCurve=" << useSCurve << "\n";
    }

    void OnUpdate()
    {
        // 归一化步长：这里假设每次 update 间隔 ~0.01s
        // 如果想更精确，可以改成通过仿真时间算 dt
        double ds = (speed * 0.01) / pathLen;

        // 更新进度
        sParam += direction * ds;

        // 到端点就反向
        if (sParam > 1.0)
        {
            sParam = 1.0;
            direction = -1;
        }
        else if (sParam < 0.0)
        {
            sParam = 0.0;
            direction = 1;
        }

        // 直线路径上的基准点（未加 S 偏移）
        double baseX = startX + (endX - startX) * sParam;
        double baseY = startY + (endY - startY) * sParam;

        double newX = baseX;
        double newY = baseY;

        if (useSCurve)
        {
            // 路径方向单位向量
            double ux = (endX - startX) / pathLen;
            double uy = (endY - startY) / pathLen;

            // 垂直于路径的单位向量（左/右方向）
            double px = -uy;
            double py = ux;

            // phase: [0,1] → 在路径上的“进度”
            double phase = sParam;

            // 让两端偏移为 0，中间左右摆动，形成 S 型
            const double PI = 3.141592653589793;
            double angle = PI * (2.0 * phase - 1.0) * sWaves; // 控制弯曲次数
            double offset = sAmplitude * std::sin(angle);

            newX = baseX + px * offset;
            newY = baseY + py * offset;
        }

        model->SetWorldPose(ignition::math::Pose3d(newX, newY, 0.3, 0, 0, 0));
    }
};

GZ_REGISTER_MODEL_PLUGIN(AutoMoveObstacle)
} // namespace gazebo

