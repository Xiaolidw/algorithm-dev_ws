#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ObstacleController : public rclcpp::Node {
public:
    ObstacleController() : Node("auto_move_obstacle") {
        RCLCPP_INFO(this->get_logger(), "🚀 ObstacleController started");
        
        // 获取参数
        this->declare_parameter<std::string>("model_name", "obstacle");
        this->declare_parameter<double>("start_x", 0.0);
        this->declare_parameter<double>("start_y", 0.0);
        this->declare_parameter<double>("end_x", 5.0);
        this->declare_parameter<double>("end_y", 0.0);
        this->declare_parameter<double>("speed", 0.05);
        this->declare_parameter<double>("update_rate", 10.0);
        
        this->get_parameter("model_name", model_name_);
        this->get_parameter("start_x", start_x_);
        this->get_parameter("start_y", start_y_);
        this->get_parameter("end_x", end_x_);
        this->get_parameter("end_y", end_y_);
        this->get_parameter("speed", speed_);
        this->get_parameter("update_rate", update_rate_);
        
        current_x_ = start_x_;
        current_y_ = start_y_;
        
        // 计算方向向量
        dx_ = end_x_ - start_x_;
        dy_ = end_y_ - start_y_;
        double distance = sqrt(dx_ * dx_ + dy_ * dy_);
        if (distance > 0) {
            dx_ /= distance;
            dy_ /= distance;
        }
        
        // 创建服务客户端
        set_model_state_client_ = this->create_client<gazebo_msgs::srv::SetModelState>("/gazebo/set_model_state");
        
        // 等待服务可用
        while (!set_model_state_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting for gazebo service...");
        }
        
        // 创建Publisher
        current_state_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/" + model_name_ + "/current_pose", 10);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/update_rate_),
            std::bind(&ObstacleController::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "✅ Obstacle %s initialized at (%.2f, %.2f)", 
                   model_name_.c_str(), start_x_, start_y_);
    }
    
private:
    void timer_callback() {
        // 更新位置
        double distance_to_end = sqrt(pow(end_x_ - current_x_, 2) + pow(end_y_ - current_y_, 2));
        
        // 如果到达终点，反转方向
        if (distance_to_end < 0.1) {
            std::swap(start_x_, end_x_);
            std::swap(start_y_, end_y_);
            dx_ = -dx_;
            dy_ = -dy_;
            RCLCPP_INFO(this->get_logger(), "🔄 %s reversed direction", model_name_.c_str());
        }
        
        // 移动
        double move_distance = speed_ / update_rate_;
        current_x_ += dx_ * move_distance;
        current_y_ += dy_ * move_distance;
        
        // 设置模型状态
        auto request = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
        request->model_state.model_name = model_name_;
        request->model_state.pose.position.x = current_x_;
        request->model_state.pose.position.y = current_y_;
        request->model_state.pose.position.z = 0.375;
        request->model_state.pose.orientation.w = 1.0;
        request->model_state.reference_frame = "world";
        
        set_model_state_client_->async_send_request(request);
        
        // 发布当前状态
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = current_x_;
        pose_msg.position.y = current_y_;
        pose_msg.position.z = 0.375;
        pose_msg.orientation.w = 1.0;
        current_state_pub_->publish(pose_msg);
    }
    
private:
    rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr set_model_state_client_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr current_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string model_name_;
    double start_x_, start_y_;
    double end_x_, end_y_;
    double speed_;
    double update_rate_;
    
    double current_x_, current_y_;
    double dx_, dy_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ObstacleController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
EOF
