#ifndef OFFBOARD_BASE_CONTROLLER_HPP
#define OFFBOARD_BASE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp> // 新增

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <functional>
#include <memory>
#include <chrono>
#include "offboard/pid_controller.hpp"
#include "msg_tool/msg/flight_info.hpp"


namespace offboard
{

enum class FlightState
{
    INIT,
    TAKEOFF,
    WAYPOINT,
    APPROACH,
    TILTLAND,
    LAND
};

// 悬停状态枚举
enum class HoverState {
    IDLE,           // 未悬停
    POSITIONING,    // 正在移动到悬停位置
    HOVERING,       // 正在悬停
    COMPLETED       // 悬停完成
};

// 悬停配置结构体
struct HoverConfig {
    double x, y, z, yaw;                    // 悬停位置和姿态
    double duration;                        // 悬停时间（秒），-1表示无限时间
    double position_tolerance;              // 位置容差
    double yaw_tolerance;                   // 偏航角容差
    bool use_current_position;              // 是否使用当前位置作为悬停点
    std::function<bool()> exit_condition;   // 退出条件函数（可选）
    
    HoverConfig() : 
        x(0.0), y(0.0), z(0.0), yaw(0.0), 
        duration(-1.0), 
        position_tolerance(0.05), 
        yaw_tolerance(0.05),
        use_current_position(true),
        exit_condition(nullptr) {}
};

class BaseController : public rclcpp::Node
{
public:
    explicit BaseController(const std::string& node_name);

    // 配置QoS
    rclcpp::QoS qos_best_effort = rclcpp::QoS(10)
        .best_effort()
        .durability_volatile();// 最佳努力传输，一直发
    rclcpp::QoS qos_reliable = rclcpp::QoS(10)
        .reliable()
        .durability_volatile();// 可靠传输，单发
    
    // 基础控制功能
    void arm();
    void engage_offboard_mode() const;
    void land();
    void auto_land();
    void task_state_pub() const;
    void publish_position_setpoint(double x, double y, double z, double yaw);
    void publish_velocity_body(double v_x, double v_y, double v_z, double v_yaw);

    
    // 优化的悬停接口
    bool start_hover(const HoverConfig& config);
    bool start_hover(double duration);  // 简化版本：在当前位置悬停指定时间
    bool start_hover_at_position(double x,double y,double z,double yaw, double duration);
    bool start_hover_with_condition(const std::function<bool()>& exit_condition);  // 条件悬停
    void stop_hover();
    bool update_hover();  // 更新悬停状态，返回是否完成
    HoverState get_hover_state() const { return hover_state_; }
    bool is_hovering() const { return hover_state_ == HoverState::HOVERING; }
    
    // 工具函数
    double distance_to_target(double x, double y, double z) const;
    static std::vector<double> get_euler_from_pose(const geometry_msgs::msg::Pose& pose);
    void rotation(int reverse);
    static double shortest_angular_distance(double from, double to);
    static std::string flightStateToString(const FlightState& state);

protected:
    // 回调函数
    virtual void timer_callback() = 0;
    void vehicle_state_callback(const mavros_msgs::msg::State::ConstSharedPtr& msg);
    void local_position_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void arm_callback(std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> future);
    void set_mode_callback(std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> future) const;
    void launch_callback(const std_msgs::msg::Bool::ConstSharedPtr& future);
    void task_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg);
    
    // 悬停相关的私有方法
    void reset_hover_state();
    bool check_hover_position_reached();
    bool check_hover_exit_conditions();

    // ROS2 接口
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_setpoint_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_; // 修正类型
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_sub_;
    rclcpp::Publisher<msg_tool::msg::FlightInfo>::SharedPtr task_pub_;



    // 状态变量
    FlightState flight_state_ = FlightState::INIT;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped local_position_;
    bool launch_flag_ = false;
    int current_task_id_ = 1; // 存储当前任务ID
    // 当前姿态
    double current_roll_ = 0, current_pitch_ = 0, current_yaw_ = 0;

    
    // 速度限制
    double max_linear_velocity_;
    double max_angular_velocity_;
    // PID 参数

    double kP_yaw_ = 1.0;             // yaw 比例增益
    double last_err_body_x_ = 0.0;
    double last_err_body_y_ = 0.0;
    double last_err_body_z_ = 0.0;
    double integral_err_body_x_ = 0.0;
    double integral_err_body_y_ = 0.0;
    double integral_err_body_z_ = 0.0;

    struct TrajectorySegment {
    double start_x, start_y, start_z;
    double end_x, end_y, end_z;
    double length;
    double direction_x, direction_y, direction_z; // 单位方向向量
};
    void publish_position_setpoint_trajectory(
        const double start_x, const double start_y, const double start_z,
        const double end_x, const double end_y, const double end_z,
        const double target_yaw) ;

    // 类成员变量
double last_cross_err_x_ = 0.0;
double last_cross_err_y_ = 0.0;
rclcpp::Time last_cross_time_ = this->get_clock()->now();


    rclcpp::Time last_time_ = this->get_clock()->now();
    
    // 悬停相关状态
    HoverState hover_state_;
    HoverConfig hover_config_;
    rclcpp::Time hover_start_time_;
    
};

} // namespace offboard

#endif // OFFBOARD_BASE_CONTROLLER_HPP