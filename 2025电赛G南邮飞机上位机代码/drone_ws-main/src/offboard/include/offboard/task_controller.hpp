#ifndef OFFBOARD_TASK_CONTROLLER_HPP
#define OFFBOARD_TASK_CONTROLLER_HPP

#include "offboard/base_controller.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include "msg_tool/msg/color.hpp"
#include <memory>

// 前向声明
class ChessboardPathPlanner;

namespace offboard
{
    class TaskController : public BaseController
    {
    public:
        TaskController();

    private:
        void timer_callback() override;
        void execute_waypoint_mission();
        bool check_task_switch_conditions();
        void switch_task(FlightState new_state);
        bool is_at_point(double x, double y, double z) const;
        bool check_emergency_condition();

        // 路径规划相关函数
        void plan_and_publish_path();

        // 任务执行函数
        bool tilt_land();

        // 状态变量
        std::vector<std::vector<double>> waypoints_;
        size_t current_waypoint_index_ = 0;
        bool setpoint_ready_ = false;
        int offboard_setpoint_counter_ = 0;
        double waypoint_threshold_;
        double takeoff_height_;


        // 路径规划相关变量
        std::unique_ptr<ChessboardPathPlanner> path_planner_;
        bool obstacles_received_ = false;
        bool path_planned_ = false;
        rclcpp::TimerBase::SharedPtr path_publish_timer_;
        rclcpp::TimerBase::SharedPtr planning_stop_timer_;
        void start_continuous_planning();
        void publish_current_best_path();
        void finalize_planning();
        void publish_path_as_waypoints(const std::vector<std::string>& path, bool is_final);
        void compress_waypoints(std::vector<std::vector<double>>& waypoints);

        // 发布者
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr processed_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr esp32_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr path_pub_;
        rclcpp::Publisher<msg_tool::msg::Color>::SharedPtr target_pose_pub_;

        // 订阅者
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr nofly_zone_sub_;
        rclcpp::Subscription<msg_tool::msg::Color>::SharedPtr target_sub_;

        // 回调函数
        void nofly_zone_callback(const geometry_msgs::msg::Polygon::ConstSharedPtr& msg);
        void target_callback(const msg_tool::msg::Color::ConstSharedPtr& msg);

        // 数据存储
        std::vector<std::pair<double, double>> nofly_zones_;

        std::string world_to_grid(double x, double y) const;
        std::string get_next_grid_cell() const;
        bool is_target_in_valid_grid(double target_world_x, double target_world_y) const;
        bool approach();

        msg_tool::msg::Color target_msg_;
        struct ProcessedTarget {
            std::string color;
            double x;
            double y;
        };
        
        geometry_msgs::msg::Polygon path_msg;

        std::vector<ProcessedTarget> processed_targets_;
        double processed_target_radius_ = 0.08;  // 认为已处理目标的判定半径
        int approach_success_count_ = 0; // 连续成功帧计数
const int APPROACH_THRESHOLD = 3; // 连续成功帧阈值

        rclcpp::Time last_position_time_;
        double last_position_x_ = 0.0;
        double last_position_y_ = 0.0;
        double last_position_z_ = 0.0;
        bool position_stuck_detected_ = false;
        rclcpp::Time stuck_detection_time_;
        rclcpp::Time ignore_target_until_;
        bool ignoring_targets_ = false;
        void reset_position_stuck_detection();
        void check_position_stuck_protection();

    };

} // namespace offboard

#endif // OFFBOARD_TASK_CONTROLLER_HPP