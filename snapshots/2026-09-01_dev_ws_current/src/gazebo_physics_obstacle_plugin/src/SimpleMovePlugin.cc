#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
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
        double endpoint_tolerance;
        double slowdown_distance;
        double stop_distance;
        double resume_distance;
        double emergency_stop_distance;
        double yield_retreat_speed;
        double yield_clear_hold_sec;
        double yield_pass_margin;
        double yield_lateral_clearance;
        double yield_hold_until;
        double route_restore_gain;
        double route_restore_speed;
        double max_cross_track_error;
        double path_length;
        double path_ux;
        double path_uy;
        double path_nx;
        double path_ny;
        bool moving_to_end;
        bool deterministic_route_lock;
        bool yielding_to_robot;
        bool yield_pass_lock_enabled;
        bool yield_wait_for_robot_pass;
        int yield_retreat_sign;
        int yield_robot_side_sign;
        
        std::string log_prefix;
        
        // ROS 2相关
        std::shared_ptr<rclcpp::Node> ros_node;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
        std::string pose_topic;
        double pose_publish_rate;
        double last_pose_publish_time;
        double motion_z;
        double last_control_time;
        
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
            this->endpoint_tolerance = std::max(
                0.01, _sdf->Get<double>("endpoint_tolerance", 0.03).first);
            this->slowdown_distance = std::max(
                this->endpoint_tolerance,
                _sdf->Get<double>("slowdown_distance", 0.30).first);
            this->stop_distance = std::max(
                0.0, _sdf->Get<double>("stop_distance", 0.90).first);
            this->resume_distance = std::max(
                this->stop_distance,
                _sdf->Get<double>("resume_distance", 1.10).first);
            this->emergency_stop_distance = std::max(
                0.0,
                _sdf->Get<double>("emergency_stop_distance", 0.78).first);
            this->yield_retreat_speed = std::max(
                0.02,
                _sdf->Get<double>("yield_retreat_speed", 0.12).first);
            this->yield_clear_hold_sec = std::max(
                0.0,
                _sdf->Get<double>("yield_clear_hold_sec", 8.0).first);
            this->yield_pass_lock_enabled = _sdf->Get<bool>(
                "yield_pass_lock_enabled", false).first;
            // Challenge obstacles may be declared as non-cooperative rails:
            // their published path is then an invariant, not a suggestion
            // that can be changed by chassis contact.  This is intentionally
            // opt-in so the lower obstacle retains ordinary physics.
            this->deterministic_route_lock = _sdf->Get<bool>(
                "deterministic_route_lock", false).first;
            this->yield_pass_margin = std::max(
                0.05,
                _sdf->Get<double>("yield_pass_margin", 0.35).first);
            // A zero value retains the original along-rail retreat.  A
            // positive value lets a rail obstacle step sideways, away from
            // the robot, when a route endpoint leaves no longitudinal room.
            this->yield_lateral_clearance = std::max(
                0.0,
                _sdf->Get<double>("yield_lateral_clearance", 0.0).first);
            this->route_restore_gain = std::max(
                0.0, _sdf->Get<double>("route_restore_gain", 2.5).first);
            this->route_restore_speed = std::max(
                0.02, _sdf->Get<double>("route_restore_speed", 0.22).first);
            this->max_cross_track_error = std::max(
                0.02,
                _sdf->Get<double>("max_cross_track_error", 0.12).first);
            const double path_dx = this->end_x - this->start_x;
            const double path_dy = this->end_y - this->start_y;
            this->path_length = std::hypot(path_dx, path_dy);
            if (this->path_length < 1e-6)
            {
                std::cerr << this->log_prefix
                          << "ERROR: moving-obstacle path is too short"
                          << std::endl;
                return;
            }
            this->path_ux = path_dx / this->path_length;
            this->path_uy = path_dy / this->path_length;
            this->path_nx = -this->path_uy;
            this->path_ny = this->path_ux;
            
            this->moving_to_end = true;
            this->yielding_to_robot = false;
            this->yield_wait_for_robot_pass = false;
            this->yield_retreat_sign = 1;
            this->yield_robot_side_sign = 1;
            this->yield_hold_until = -1.0;
            
            // 输出参数信息
            std::cout << this->log_prefix << "Parameters:" << std::endl;
            std::cout << this->log_prefix << "  Speed: " << this->speed << " m/s" << std::endl;
            std::cout << this->log_prefix << "  Force: " << this->force << " N" << std::endl;
            std::cout << this->log_prefix << "  Pose publish rate: "
                      << this->pose_publish_rate << " Hz" << std::endl;
            std::cout << this->log_prefix << "  Path: (" << this->start_x << "," << this->start_y 
                      << ") -> (" << this->end_x << "," << this->end_y << ")" << std::endl;
            std::cout << this->log_prefix << "  Endpoint tolerance: "
                      << this->endpoint_tolerance << " m | robot yield: "
                      << this->stop_distance << "/" << this->resume_distance
                      << " m, retreat=" << this->yield_retreat_speed
                      << " m/s, clear-hold=" << this->yield_clear_hold_sec
                      << " s" << std::endl;
            std::cout << this->log_prefix << "  Yield pass lock: "
                      << (this->yield_pass_lock_enabled ? "enabled" : "disabled")
                      << ", margin=" << this->yield_pass_margin << " m"
                      << std::endl;
            if (this->yield_lateral_clearance > 0.0) {
                std::cout << this->log_prefix
                          << "  Yield lateral clearance: "
                          << this->yield_lateral_clearance << " m"
                          << std::endl;
            }
            std::cout << this->log_prefix << "  Rail guard: max lateral="
                      << this->max_cross_track_error << " m, restore="
                      << this->route_restore_speed << " m/s" << std::endl;
            std::cout << this->log_prefix << "  Deterministic route lock: "
                      << (this->deterministic_route_lock ? "enabled" : "disabled")
                      << std::endl;
            
            // 设置阻尼
            // 速度伺服本身负责平移稳定性。Gazebo在高频物理步进下会
            // 反复应用线性阻尼，0.2会把目标速度压到约0.016 m/s。
            this->link->SetLinearDamping(0.0);
            this->link->SetAngularDamping(0.2);
            // 使用有限质量动力学。碰撞冲量会由 Gazebo 根据 SDF 中的
            // mass / inertia 正常求解，不能再用 Kinematic + SetWorldPose
            // 强制穿过机器人。
            this->link->SetKinematic(this->deterministic_route_lock);
            // 轻质量障碍物沿固定高度导轨运动，避免地面接触扰动训练速度。
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
                // 获取当前状态
                ignition::math::Pose3d pose = this->model->WorldPose();
                ignition::math::Vector3d linear_vel = this->link->WorldLinearVel();
                double current_time = _info.simTime.Double();
                double control_dt = this->last_control_time < 0.0
                    ? 0.0
                    : current_time - this->last_control_time;
                this->last_control_time = current_time;
                control_dt = std::max(0.0, std::min(control_dt, 0.05));
                
                double current_x = pose.Pos().X();
                double current_y = pose.Pos().Y();
                
                // Keep physics updates at Gazebo rate, but publish training and
                // prediction input poses at a stable, configurable rate.
                const double pose_publish_period = 1.0 / this->pose_publish_rate;
                if (this->last_pose_publish_time < 0.0 ||
                    current_time < this->last_pose_publish_time ||
                    current_time - this->last_pose_publish_time >=
                        pose_publish_period - 1e-9)
                {
                    this->PublishPose(pose);
                    this->last_pose_publish_time = current_time;
                }

                // The upper competition obstacle is required to patrol the
                // fixed [-3, 2] x y=3 rail at its declared speed even after a
                // chassis touches it.  A finite-mass velocity servo can be
                // pushed off the rail, leaving prediction at 0.20 m/s while
                // the physical model is nearly stationary.  Drive this
                // explicitly kinematic model by route arc-length instead;
                // it keeps its collision geometry but cannot be displaced or
                // "yield" to the robot.  Robot-side Collision Monitor must
                // stop before contact, so this never acts as a teleporting
                // avoidance mechanism.
                if (this->deterministic_route_lock) {
                    if (control_dt <= 0.0) {
                        this->link->SetLinearVel(
                            ignition::math::Vector3d::Zero);
                        return;
                    }
                    double along = (current_x - this->start_x) * this->path_ux +
                        (current_y - this->start_y) * this->path_uy;
                    along = std::max(0.0, std::min(this->path_length, along));
                    const double direction = this->moving_to_end ? 1.0 : -1.0;
                    double next = along + direction * this->speed * control_dt;
                    if (next >= this->path_length) {
                        next = this->path_length;
                        this->SwitchDirection();
                    } else if (next <= 0.0) {
                        next = 0.0;
                        this->SwitchDirection();
                    }
                    pose.Pos().X(this->start_x + this->path_ux * next);
                    pose.Pos().Y(this->start_y + this->path_uy * next);
                    pose.Pos().Z(this->motion_z);
                    this->model->SetWorldPose(pose);
                    const double next_direction =
                        this->moving_to_end ? 1.0 : -1.0;
                    this->link->SetLinearVel(ignition::math::Vector3d(
                        next_direction * this->path_ux * this->speed,
                        next_direction * this->path_uy * this->speed, 0.0));
                    this->link->SetAngularVel(ignition::math::Vector3d::Zero);
                    static double locked_last_print_time = 0.0;
                    if (current_time - locked_last_print_time >= 5.0) {
                        this->PrintStatus(current_time, pose,
                            this->link->WorldLinearVel(),
                            std::abs((this->moving_to_end ? this->path_length : 0.0)
                                     - next));
                        locked_last_print_time = current_time;
                    }
                    return;
                }

                // 机器人接近并且双方仍在靠近时主动沿导轨退离。原来的
                // 原地停车会在巡逻端点封死通道，让 Nav2 与障碍物永久互等。
                // 极近距离仍保持紧急停车，退离只在导轨范围内进行。
                double robot_distance = std::numeric_limits<double>::infinity();
                {
                    gazebo::physics::ModelPtr robot =
                        this->model->GetWorld()->ModelByName("six_arm");
                    if (robot) {
                        ignition::math::Pose3d robot_pose = robot->WorldPose();
                        double rx = current_x - robot_pose.Pos().X();
                        double ry = current_y - robot_pose.Pos().Y();
                        robot_distance = sqrt(rx*rx + ry*ry);
                        ignition::math::Vector3d robot_velocity =
                            robot->WorldLinearVel();
                        double closing_speed = 0.0;
                        if (robot_distance > 1e-6) {
                            const double relative_vx =
                                linear_vel.X() - robot_velocity.X();
                            const double relative_vy =
                                linear_vel.Y() - robot_velocity.Y();
                            const double distance_rate =
                                (rx * relative_vx + ry * relative_vy) /
                                robot_distance;
                            closing_speed = -distance_rate;
                        }
                        const bool was_yielding = this->yielding_to_robot;
                        const double current_t =
                            (current_x - this->start_x) * this->path_ux +
                            (current_y - this->start_y) * this->path_uy;
                        const double robot_t =
                            (robot_pose.Pos().X() - this->start_x) *
                                this->path_ux +
                            (robot_pose.Pos().Y() - this->start_y) *
                                this->path_uy;
                        const double robot_cross_track =
                            (robot_pose.Pos().X() - this->start_x) *
                                this->path_nx +
                            (robot_pose.Pos().Y() - this->start_y) *
                                this->path_ny;
                        const bool entering_yield =
                            robot_distance <= this->stop_distance &&
                            (closing_speed > 0.02 ||
                             robot_distance <= this->emergency_stop_distance);

                        if (this->stop_distance <= 0.0) {
                            this->yielding_to_robot = false;
                            this->yield_wait_for_robot_pass = false;
                        } else if (this->yield_pass_lock_enabled) {
                            if (!this->yielding_to_robot &&
                                !this->yield_wait_for_robot_pass &&
                                entering_yield) {
                                // Choose the side away from the robot once and
                                // retain it for the entire yield cycle.  This
                                // avoids the old distance-only release, which
                                // could immediately send the obstacle back into
                                // the robot's crossing corridor.
                                this->yield_retreat_sign =
                                    robot_t <= current_t ? 1 : -1;
                                this->yield_robot_side_sign =
                                    robot_cross_track >= 0.0 ? 1 : -1;
                                this->yielding_to_robot = true;
                                this->yield_hold_until = -1.0;
                            } else if (this->yielding_to_robot) {
                                // A robot can cross a rail laterally, or it can
                                // pass one of its endpoints while remaining on
                                // the same side.  The latter is the red-1
                                // approach: requiring only a lateral crossing
                                // held the obstacle at START directly in the
                                // robot's corridor forever.
                                const bool crossed_rail =
                                    this->yield_robot_side_sign *
                                        robot_cross_track <=
                                    -this->yield_pass_margin;
                                const bool passed_endpoint =
                                    this->yield_retreat_sign < 0
                                        ? robot_t <= current_t -
                                            this->yield_pass_margin
                                        : robot_t >= current_t +
                                            this->yield_pass_margin;
                                const bool robot_passed =
                                    crossed_rail || passed_endpoint;
                                if (robot_passed &&
                                    robot_distance >= this->resume_distance) {
                                    this->yielding_to_robot = false;
                                    this->yield_wait_for_robot_pass = false;
                                    this->moving_to_end =
                                        this->yield_retreat_sign < 0;
                                    std::cout << this->log_prefix
                                              << "Yield pass lock released after rail crossing at robot distance "
                                              << robot_distance << " m"
                                              << std::endl;
                                }
                            }
                        } else {
                            if (!this->yielding_to_robot && entering_yield) {
                                this->yielding_to_robot = true;
                                this->yield_hold_until = -1.0;
                            } else if (this->yielding_to_robot &&
                                       robot_distance >= this->resume_distance) {
                                this->yielding_to_robot = false;
                                this->yield_hold_until =
                                    current_time + this->yield_clear_hold_sec;
                            }
                        }
                        if (!was_yielding && this->yielding_to_robot) {
                            std::cout << this->log_prefix
                                      << "Yield retreat activated at robot distance "
                                      << robot_distance << " m"
                                      << (this->yield_pass_lock_enabled
                                          ? (this->yield_retreat_sign > 0
                                              ? "; direction locked toward END"
                                              : "; direction locked toward START")
                                          : "")
                                      << std::endl;
                        } else if (was_yielding &&
                                   !this->yielding_to_robot) {
                            std::cout << this->log_prefix
                                      << (this->yield_wait_for_robot_pass
                                          ? "Yield clearance reached; holding rail until robot passes"
                                          : "Yield retreat cleared")
                                      << " at robot distance " << robot_distance
                                      << " m" << std::endl;
                        }
                        if (this->yielding_to_robot) {
                            const double retreat_sign =
                                this->yield_pass_lock_enabled
                                    ? static_cast<double>(this->yield_retreat_sign)
                                    : (robot_t <= current_t ? 1.0 : -1.0);
                            const bool route_has_room =
                                (retreat_sign > 0.0 &&
                                 current_t < this->path_length -
                                     this->endpoint_tolerance) ||
                                (retreat_sign < 0.0 &&
                                 current_t > this->endpoint_tolerance);
                            // A sideways step is an endpoint escape hatch,
                            // not the normal yield path.  Using it in the
                            // middle of the rail can leave the obstacle on
                            // the opposite side of a robot that has not yet
                            // crossed, causing both to wait forever.
                            if (this->yield_lateral_clearance > 0.0 &&
                                !route_has_room) {
                                // At a patrol endpoint an along-rail retreat
                                // has no free distance.  Move perpendicular to
                                // the rail, explicitly away from the robot,
                                // and hold that latched side until the robot
                                // has crossed or passed the endpoint.  This
                                // creates a real safety gap without weakening
                                // Collision Monitor or any map inflation.
                                const double cross_track =
                                    (current_x - this->start_x) * this->path_nx +
                                    (current_y - this->start_y) * this->path_ny;
                                const double target_cross_track =
                                    -static_cast<double>(
                                        this->yield_robot_side_sign) *
                                    this->yield_lateral_clearance;
                                const double cross_error =
                                    target_cross_track - cross_track;
                                const double lateral_speed =
                                    std::max(-this->yield_retreat_speed,
                                             std::min(this->yield_retreat_speed,
                                                      cross_error * 2.0));
                                this->link->SetLinearVel(
                                    ignition::math::Vector3d(
                                        this->path_nx * lateral_speed,
                                        this->path_ny * lateral_speed,
                                        0.0));
                                this->link->SetAngularVel(
                                    ignition::math::Vector3d::Zero);
                                return;
                            }
                            if (!route_has_room || control_dt <= 0.0) {
                                this->link->SetLinearVel(
                                    ignition::math::Vector3d(0, 0, 0));
                            } else {
                                this->moving_to_end = retreat_sign > 0.0;
                                ignition::math::Vector3d desired_retreat(
                                    retreat_sign * this->path_ux *
                                        this->yield_retreat_speed,
                                    retreat_sign * this->path_uy *
                                        this->yield_retreat_speed,
                                    0.0);
                                ignition::math::Vector3d correction(
                                    desired_retreat.X() - linear_vel.X(),
                                    desired_retreat.Y() - linear_vel.Y(),
                                    0.0);
                                const double mass = std::max(
                                    0.1, this->link->GetInertial()->Mass());
                                const double max_step =
                                    (this->force / mass) * control_dt;
                                const double correction_norm = std::hypot(
                                    correction.X(), correction.Y());
                                if (correction_norm > max_step &&
                                    correction_norm > 1e-9) {
                                    correction *= max_step / correction_norm;
                                }
                                this->link->SetLinearVel(
                                    ignition::math::Vector3d(
                                        linear_vel.X() + correction.X(),
                                        linear_vel.Y() + correction.Y(),
                                        0.0));
                            }
                            this->link->SetAngularVel(
                                ignition::math::Vector3d::Zero);
                            return;
                        }
                        if (this->yield_pass_lock_enabled &&
                            this->yield_wait_for_robot_pass) {
                            const bool robot_passed =
                                this->yield_retreat_sign < 0
                                    ? robot_t <= current_t - this->yield_pass_margin
                                    : robot_t >= current_t + this->yield_pass_margin;
                            if (robot_passed &&
                                robot_distance >= this->resume_distance) {
                                // The robot is now beyond the held obstacle on
                                // the rail and outside the hysteresis radius.
                                // Only now may patrol reverse from the locked
                                // retreat direction.
                                this->yield_wait_for_robot_pass = false;
                                this->moving_to_end =
                                    this->yield_retreat_sign < 0;
                                std::cout << this->log_prefix
                                          << "Yield pass lock released at robot distance "
                                          << robot_distance << " m" << std::endl;
                            } else {
                                this->link->SetLinearVel(
                                    ignition::math::Vector3d::Zero);
                                this->link->SetAngularVel(
                                    ignition::math::Vector3d::Zero);
                                return;
                            }
                        }
                        // Releasing exactly at the hysteresis boundary and
                        // immediately resuming patrol can send the obstacle
                        // straight back toward the robot.  Hold the cleared
                        // rail position briefly so Nav2 has a real crossing
                        // window.  Hard emergency/yield checks above remain
                        // active throughout the hold.
                        if (!this->yield_pass_lock_enabled &&
                            this->yield_hold_until > current_time) {
                            this->link->SetLinearVel(
                                ignition::math::Vector3d::Zero);
                            this->link->SetAngularVel(
                                ignition::math::Vector3d::Zero);
                            return;
                        }
                        if (!this->yield_pass_lock_enabled &&
                            this->yield_hold_until >= 0.0) {
                            std::cout << this->log_prefix
                                      << "Yield clear hold completed"
                                      << std::endl;
                            this->yield_hold_until = -1.0;
                        }
                    }
                }

                // 物理碰撞可以短时推开轻量障碍物，但不能永久改变已公布的
                // 巡逻路线。机器人离开安全区后，按受限速度平滑投影回导轨；
                // 不在接触附近纠偏，避免任何 SetWorldPose 穿过机器人。
                double cross_track =
                    (current_x - this->start_x) * this->path_nx +
                    (current_y - this->start_y) * this->path_ny;
                if (std::abs(cross_track) > this->max_cross_track_error &&
                    robot_distance >= this->resume_distance &&
                    control_dt > 0.0)
                {
                    const double correction = std::copysign(
                        std::min(
                            std::abs(cross_track) -
                                this->max_cross_track_error,
                            this->route_restore_speed * control_dt),
                        cross_track);
                    pose.Pos().X(current_x - correction * this->path_nx);
                    pose.Pos().Y(current_y - correction * this->path_ny);
                    pose.Pos().Z(this->motion_z);
                    this->model->SetWorldPose(pose);
                    current_x = pose.Pos().X();
                    current_y = pose.Pos().Y();
                    cross_track -= correction;
                }

                // Contacts can push the light physical obstacle beyond a
                // patrol endpoint.  `moving_to_end` only expresses the last
                // intended patrol direction; when the model is already past
                // that endpoint, retaining the old sign drives it farther
                // away forever.  Re-enter the rail under velocity control
                // (rather than teleporting through a nearby robot).
                const double along_track =
                    (current_x - this->start_x) * this->path_ux +
                    (current_y - this->start_y) * this->path_uy;
                if (along_track < -this->endpoint_tolerance &&
                    !this->moving_to_end)
                {
                    this->moving_to_end = true;
                    std::cout << this->log_prefix
                              << "Patrol re-entry: recovered beyond START"
                              << std::endl;
                }
                else if (along_track > this->path_length +
                                           this->endpoint_tolerance &&
                         this->moving_to_end)
                {
                    this->moving_to_end = false;
                    std::cout << this->log_prefix
                              << "Patrol re-entry: recovered beyond END"
                              << std::endl;
                }

                // 确定目标点
                double target_x = this->moving_to_end ? this->end_x : this->start_x;
                double target_y = this->moving_to_end ? this->end_y : this->start_y;
                
                // 计算到目标的距离和方向
                double dx = target_x - current_x;
                double dy = target_y - current_y;
                double distance = sqrt(dx*dx + dy*dy);
                
                // 检查是否到达目标
                if (distance <= this->endpoint_tolerance)
                {
                    this->SwitchDirection();
                    target_x = this->moving_to_end ? this->end_x : this->start_x;
                    target_y = this->moving_to_end ? this->end_y : this->start_y;
                    dx = target_x - current_x;
                    dy = target_y - current_y;
                    distance = sqrt(dx*dx + dy*dy);
                }
                
                // 如果距离太小，不施加力
                if (distance <= this->endpoint_tolerance)
                {
                    this->PrintStatus(_info.simTime.Double(), pose, linear_vel, 0.0);
                    return;
                }
                
                // 保留国赛方案的轻质量和高恢复能力，同时用受15 N上限
                // 约束的速度伺服精确保持LSTM训练时的0.25 m/s。
                const double commanded_speed = std::min(
                    this->speed,
                    std::max(0.04, this->speed * distance /
                        this->slowdown_distance));
                const double travel_sign = this->moving_to_end ? 1.0 : -1.0;
                const double lateral_speed = std::max(
                    -this->route_restore_speed,
                    std::min(
                        this->route_restore_speed,
                        -this->route_restore_gain * cross_track));
                ignition::math::Vector3d desired_velocity(
                    travel_sign * this->path_ux * commanded_speed +
                        this->path_nx * lateral_speed,
                    travel_sign * this->path_uy * commanded_speed +
                        this->path_ny * lateral_speed,
                    0.0);
                ignition::math::Vector3d velocity_correction(
                    desired_velocity.X() - linear_vel.X(),
                    desired_velocity.Y() - linear_vel.Y(),
                    0.0);
                const double mass = std::max(
                    0.1,
                    this->link->GetInertial()->Mass());
                const double correction_norm = std::hypot(
                    velocity_correction.X(),
                    velocity_correction.Y());
                const double max_velocity_step =
                    (this->force / mass) * control_dt;
                if (correction_norm > max_velocity_step &&
                    correction_norm > 1e-9)
                {
                    velocity_correction *=
                        max_velocity_step / correction_norm;
                }
                this->link->SetLinearVel(
                    ignition::math::Vector3d(
                        linear_vel.X() + velocity_correction.X(),
                        linear_vel.Y() + velocity_correction.Y(),
                        std::max(-0.25, std::min(
                            0.25,
                            (this->motion_z - pose.Pos().Z()) * 4.0))));
                // Obstacles are planar moving references, not spinning rigid
                // bodies.  Removing collision-induced angular momentum keeps
                // their contact geometry predictable and prevents a hit from
                // converting into an upward launch of the mobile base.
                this->link->SetAngularVel(ignition::math::Vector3d::Zero);
                
                // 定期输出状态
                static double last_print_time = 0;
                if (current_time - last_print_time >= 5.0)
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
            this->updateConnection.reset();
            this->pose_publisher.reset();
            this->ros_node.reset();
        }
    };
    
    GZ_REGISTER_MODEL_PLUGIN(SimpleMovePlugin)
}
