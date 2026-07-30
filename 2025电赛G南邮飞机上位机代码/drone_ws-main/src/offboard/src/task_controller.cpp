#include <Eigen/Core>
#include "offboard/task_controller.hpp"
#include "offboard/ChessboardPathPlanner.hpp"

namespace offboard
{
    TaskController::TaskController()
    : BaseController("offb_node"), path_planner_(nullptr), obstacles_received_(false), path_planned_(false)
    {
        // 声明参数
        this->declare_parameter("takeoff_height", 1.0);
        this->declare_parameter("waypoint_threshold", 0.1);

        takeoff_height_ = this->get_parameter("takeoff_height").as_double();
        waypoint_threshold_ = this->get_parameter("waypoint_threshold").as_double();
        
        // 初始化位置停留保护机制变量
        last_position_time_ = this->get_clock()->now();
        last_position_x_ = 0.0;
        last_position_y_ = 0.0;
        last_position_z_ = 0.0;
        position_stuck_detected_ = false;
        ignoring_targets_ = false;

        // 初始化路径规划器 (9x7棋盘)
        path_planner_ = std::make_unique<ChessboardPathPlanner>(9, 7);

        // 初始化发布者和订阅者
        esp32_pub_ = create_publisher<std_msgs::msg::Int32>("/esp32", qos_reliable);
        processed_pub_ = create_publisher<std_msgs::msg::Bool>("/processed_status", qos_reliable);

        nofly_zone_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
            "/nofly_zone", qos_reliable,
            [this](const geometry_msgs::msg::Polygon::ConstSharedPtr& msg){
                nofly_zone_callback(msg);
            });

        target_sub_ = create_subscription<msg_tool::msg::Color>(
            "/target", qos_reliable,
            [this](const msg_tool::msg::Color ::ConstSharedPtr& msg){
                target_callback(msg);
            });

        target_pose_pub_ = create_publisher<msg_tool::msg::Color>(
                "/target_pose",qos_reliable);

        path_pub_ = create_publisher<geometry_msgs::msg::Polygon>("/planner_path", qos_reliable);

        RCLCPP_INFO(get_logger(), "任务控制器初始化完成");
    }

    void TaskController::nofly_zone_callback(const geometry_msgs::msg::Polygon::ConstSharedPtr& msg)
    {
        nofly_zones_.clear();
        std::vector<std::string> obstacles;

        for (const auto& point : msg->points) {
            nofly_zones_.emplace_back(point.x, point.y);
            // 将坐标转换为棋盘格式 (假设点坐标是整数格式)
            int grid_x = static_cast<int>(point.x);
            int grid_y = static_cast<int>(point.y);
            if (grid_x >= 1 && grid_x <= 9 && grid_y >= 1 && grid_y <= 7) {
                std::string obstacle = "A" + std::to_string(grid_x) + "B" + std::to_string(grid_y);
                obstacles.push_back(obstacle);
            }
        }

        // 设置路径规划的障碍物
        if (path_planner_) {
            path_planner_->setObstacles(obstacles);
            RCLCPP_INFO(get_logger(), "设置了 %zu 个障碍物", obstacles.size());
        }
        RCLCPP_INFO(get_logger(), "接收到禁飞区域数据，包含 %zu 个点", nofly_zones_.size());

        // 接收到障碍物数据后立即开始持续路径规划
        start_continuous_planning();
    }

    void TaskController::start_continuous_planning()
    {
        if (!path_planner_) {
            RCLCPP_ERROR(get_logger(), "路径规划器未初始化！");
            return;
        }

        // 停止之前的规划（如果有的话）
        if (path_planner_->isPlanning()) {
            path_planner_->stopPlanning();
        }

        // 设置起始点为A9B1 (棋盘左下角)
        path_planner_->setStart("A9B1");

        // 开始持续规划
        path_planner_->startPlanning();
        RCLCPP_INFO(get_logger(), "开始持续路径规划...");

        // 创建定时器，每500ms发布一次当前最优路径
        if (path_publish_timer_) {
            path_publish_timer_->cancel();
        }
        path_publish_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&TaskController::publish_current_best_path, this)
        );

        // 创建定时器，5秒后停止规划并确定最终路径
        if (planning_stop_timer_) {
            planning_stop_timer_->cancel();
        }
        planning_stop_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() {
                    this->finalize_planning();
                    this->planning_stop_timer_->cancel();  // 只执行一次
                }
        );
    }

    void TaskController::publish_current_best_path()
    {
        if (!path_planner_ || !path_planner_->isPlanning()) {
            return;
        }

        // 获取当前最优路径
        std::vector<std::string> path = path_planner_->getPath();

        if (path.empty()) {
            RCLCPP_WARN(get_logger(), "当前无可用路径");
            return;
        }

        // 获取规划统计信息
        auto stats = path_planner_->getStats();
        RCLCPP_INFO(get_logger(), "规划状态 - 迭代: %d, 路径长度: %d, 代价: %.2f, 重复: %d, 转弯: %d",
                    stats.iteration, stats.path_length, stats.best_cost,
                    stats.repeat_count, stats.turn_count);

        // 转换路径为航点坐标并发布
        publish_path_as_waypoints(path, false);  // false表示不是最终路径，不保存到waypoints_
    }

    void TaskController::finalize_planning()
    {
        if (!path_planner_) {
            return;
        }

        // 停止持续规划
        if (path_planner_->isPlanning()) {
            path_planner_->stopPlanning();
            RCLCPP_INFO(get_logger(), "停止持续规划，确定最优路径");
        }

        // 停止路径发布定时器
        if (path_publish_timer_) {
            path_publish_timer_->cancel();
        }

        // 获取最终最优路径
        std::vector<std::string> final_path = path_planner_->getPath();

        if (final_path.empty()) {
            RCLCPP_ERROR(get_logger(), "最终路径规划失败！");
            return;
        }

        // 获取最终统计信息
        auto final_stats = path_planner_->getStats();
        RCLCPP_INFO(get_logger(), "最终路径确定 - 迭代: %d, 路径长度: %d, 代价: %.2f, 重复: %d, 转弯: %d",
                    final_stats.iteration, final_stats.path_length, final_stats.best_cost,
                    final_stats.repeat_count, final_stats.turn_count);



        // 转换并保存最终路径，同时发布（包含起点）
        publish_path_as_waypoints(final_path, true);  // true表示是最终路径，保存到waypoints_


        obstacles_received_ = true;
        path_planned_ = true;
        RCLCPP_INFO(get_logger(), "最终路径确定完成，生成 %zu 个航点", waypoints_.size());



        // 打印路径信息
        path_planner_->printPath();

        compress_waypoints(waypoints_);
    }

    void TaskController::publish_path_as_waypoints(const std::vector<std::string>& path, bool is_final)
    {
        std::vector<std::vector<double>> current_waypoints;
        geometry_msgs::msg::Polygon current_path_msg;
        current_path_msg.points.clear();

        // 转换路径为航点坐标
        for (const auto& waypoint_str : path) {
            try {
                size_t b_pos = waypoint_str.find('B');
                if (waypoint_str.empty() || waypoint_str[0] != 'A' || b_pos == std::string::npos) {
                    RCLCPP_WARN(get_logger(), "无效的航点格式: %s", waypoint_str.c_str());
                    continue;
                }

                std::string a_part = waypoint_str.substr(1, b_pos - 1);  // col number
                std::string b_part = waypoint_str.substr(b_pos + 1);     // row number
                int col = std::stoi(a_part);
                int row = std::stoi(b_part);

                // 验证范围
                if (col < 1 || col > 9) {
                    RCLCPP_WARN(get_logger(), "列超出范围 [1,9]: %s", waypoint_str.c_str());
                    continue;
                }
                if (row < 1 || row > 7) {
                    RCLCPP_WARN(get_logger(), "行超出范围 [1,7]: %s", waypoint_str.c_str());
                    continue;
                }

                // 坐标转换：
                // A 编号从 9（右）到 1（左），x = (9 - col) * 0.5
                // B 编号从 1（近）到 7（远），y = (row - 1) * 0.5
                double x = (row - 1) * 0.5;
                double y = (9 - col) * 0.5;
                double z = takeoff_height_;
                double yaw = 0.0;

                current_waypoints.push_back({x, y, z, yaw});

                geometry_msgs::msg::Point32 point;
                point.x = x;
                point.y = y;
                point.z = z;
                current_path_msg.points.push_back(point);

            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "解析航点失败 %s: %s", waypoint_str.c_str(), e.what());
            }
        }

        // 如果是最终路径，在末尾加上起点 A9B1 -> (0, 0, takeoff_height_)
        if (is_final && !current_waypoints.empty()) {
            double start_x = 0.0;  // A9B1 对应 (0, 0)
            double start_y = 0.0;
            double start_z = takeoff_height_;
            double start_yaw = 0.0;

            // current_waypoints.push_back({start_x, start_y, start_z, start_yaw});

            geometry_msgs::msg::Point32 start_point;
            start_point.x = start_x;
            start_point.y = start_y;
            start_point.z = start_z;
            current_path_msg.points.push_back(start_point);

            RCLCPP_INFO(get_logger(), "已将起点 A9B1 (%.2f, %.2f, %.2f) 添加到路径末尾",
                        start_x, start_y, start_z);
        }

        // 发布路径
        if (!current_path_msg.points.empty()) {
            path_pub_->publish(current_path_msg);

            if (is_final) {
                // 保存最终航点到成员变量
                waypoints_ = current_waypoints;
                // if (!waypoints_.empty()) {
                //     // waypoints_.pop_back(); // 删除最后一个航点

                // }
                path_msg = current_path_msg;  // 保存最终路径消息
                RCLCPP_INFO(get_logger(), "最终路径已保存，包含 %zu 个航点", waypoints_.size());
            } else {
                RCLCPP_DEBUG(get_logger(), "发布当前最优路径，包含 %zu 个航点", current_waypoints.size());
            }
        }
    }

// 航点压缩函数：合并直线航点，保留拐角处的航点，长直线段添加中点
void TaskController::compress_waypoints(std::vector<std::vector<double>>& waypoints)
{
    if (waypoints.size() < 2) {
        RCLCPP_INFO(get_logger(), "航点数量少于2个，无需压缩，保持原有 %zu 个航点", waypoints.size());
        return;
    }
    
    // 如果只有2个航点，直接返回
    if (waypoints.size() == 2) {
        RCLCPP_INFO(get_logger(), "只有2个航点，无需压缩，保持原有航点");
        return;
    }

    // 记录原始航点数量
    size_t original_count = waypoints.size();
    std::vector<std::vector<double>> raw_waypoints = waypoints;  // 保存原始数据
    waypoints.clear();  // 清空原数组，重新填充

    // 总是保留第一个点
    waypoints.push_back(raw_waypoints[0]);
    RCLCPP_INFO(get_logger(), "保留起始点: (%.2f, %.2f, %.2f)", 
            raw_waypoints[0][0], raw_waypoints[0][1], raw_waypoints[0][2]);

    // 用于记录当前直线段的信息
    size_t line_start_idx = 0;  // 当前直线段的起始索引
    std::vector<size_t> current_line_points;  // 当前直线段包含的所有点索引

    for (size_t i = 1; i < raw_waypoints.size() - 1; ++i) {
        // 获取三个连续点
        const auto& prev = raw_waypoints[i - 1];
        const auto& curr = raw_waypoints[i];
        const auto& next = raw_waypoints[i + 1];
        
        // 计算两个方向向量
        double dx1 = curr[0] - prev[0];
        double dy1 = curr[1] - prev[1];
        double dx2 = next[0] - curr[0];
        double dy2 = next[1] - curr[1];
        
        // 判断是否在同一直线上（方向向量平行）
        // 使用叉积判断：如果叉积为0，则向量平行
        double cross_product = dx1 * dy2 - dy1 * dx2;
        const double epsilon = 1e-6;  // 数值误差容忍度
        
        // 首先检查是否为掉头点（同一直线上但方向相反）
        bool is_uturn = false;
        if (std::abs(cross_product) <= epsilon) {
            // 在同一直线上，检查是否为掉头
            double dot_product = dx1 * dx2 + dy1 * dy2;
            if (dot_product < -epsilon) {
                is_uturn = true;
            }
        }
        
        // 判断是否需要保留此点（拐角点或掉头点）
        if (std::abs(cross_product) > epsilon || is_uturn) {
            // 这是拐角点或掉头点，需要保留
            
            // 先处理之前累积的直线段
            if (!current_line_points.empty()) {
                size_t total_merged_points = current_line_points.size();
                if (total_merged_points >= 4) {
                    // 计算直线段的中点
                    const auto& line_start = raw_waypoints[line_start_idx];
                    const auto& line_end = raw_waypoints[i - 1];
                    
                    // 创建中点航点
                    std::vector<double> midpoint(4);  // x, y, z, yaw
                    midpoint[0] = (line_start[0] + line_end[0]) / 2.0;
                    midpoint[1] = (line_start[1] + line_end[1]) / 2.0;
                    midpoint[2] = (line_start[2] + line_end[2]) / 2.0;
                    midpoint[3] = line_start.size() > 3 ? line_start[3] : 0.0;  // yaw
                    
                    waypoints.push_back(midpoint);
                    RCLCPP_INFO(get_logger(), "添加长直线段中点: (%.2f, %.2f, %.2f), 合并了%zu个点", 
                            midpoint[0], midpoint[1], midpoint[2], total_merged_points);
                }
                
                RCLCPP_INFO(get_logger(), "完成直线段合并: 起始索引%zu, 合并了%zu个点", 
                        line_start_idx, total_merged_points);
                current_line_points.clear();
            }
            
            // 保留当前关键点
            waypoints.push_back(curr);
            if (is_uturn) {
                double dot_product = dx1 * dx2 + dy1 * dy2;
                RCLCPP_INFO(get_logger(), "保留掉头点: (%.2f, %.2f, %.2f), 点积: %.6f", 
                        curr[0], curr[1], curr[2], dot_product);
            } else {
                RCLCPP_INFO(get_logger(), "保留拐角点: (%.2f, %.2f, %.2f), 叉积: %.6f", 
                        curr[0], curr[1], curr[2], cross_product);
            }
            
            // 重置直线段记录
            line_start_idx = i;
            
        } else {
            // 普通直线点，添加到当前直线段
            if (current_line_points.empty()) {
                // 开始新的直线段
                line_start_idx = i - 1;  // 直线段从前一个点开始
            }
            current_line_points.push_back(i);
            
            RCLCPP_DEBUG(get_logger(), "跳过直线点: (%.2f, %.2f, %.2f), 叉积: %.6f", 
                    curr[0], curr[1], curr[2], cross_product);
        }
    }

    // 处理最后可能剩余的直线段
    if (!current_line_points.empty()) {
        size_t total_merged_points = current_line_points.size();
        if (total_merged_points >= 4) {
            // 计算直线段的中点
            const auto& line_start = raw_waypoints[line_start_idx];
            const auto& line_end = raw_waypoints[raw_waypoints.size() - 2];  // 倒数第二个点
            
            // 创建中点航点
            std::vector<double> midpoint(4);
            midpoint[0] = (line_start[0] + line_end[0]) / 2.0;
            midpoint[1] = (line_start[1] + line_end[1]) / 2.0;
            midpoint[2] = (line_start[2] + line_end[2]) / 2.0;
            midpoint[3] = line_start.size() > 3 ? line_start[3] : 0.0;
            
            waypoints.push_back(midpoint);
            RCLCPP_INFO(get_logger(), "添加最后直线段中点: (%.2f, %.2f, %.2f), 合并了%zu个点", 
                    midpoint[0], midpoint[1], midpoint[2], total_merged_points);
        }
        
        RCLCPP_INFO(get_logger(), "完成最后直线段合并: 起始索引%zu, 合并了%zu个点", 
                line_start_idx, total_merged_points);
    }

    // 总是保留最后一个点
    if (raw_waypoints.size() > 1) {
        waypoints.push_back(raw_waypoints.back());
        RCLCPP_INFO(get_logger(), "保留终点: (%.2f, %.2f, %.2f)", 
                raw_waypoints.back()[0], raw_waypoints.back()[1], raw_waypoints.back()[2]);
    }
    
    RCLCPP_INFO(get_logger(), "航点压缩完成：原始 %zu 个航点 -> 压缩后 %zu 个航点", 
                original_count, waypoints.size());
        // 输出合并后的航线
    std::stringstream compressed_path_ss;
    compressed_path_ss << "压缩后航线: ";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        // 将坐标转换回航点名称
        double x = waypoints[i][0];
        double y = waypoints[i][1];
        
        // 逆向转换坐标到航点名称
        // x = (row - 1) * 0.5 -> row = x / 0.5 + 1
        // y = (9 - col) * 0.5 -> col = 9 - y / 0.5
        int row = static_cast<int>(std::round(x / 0.5 + 1));
        int col = static_cast<int>(std::round(9 - y / 0.5));
        
        // 验证范围并格式化输出
        if (row >= 1 && row <= 7 && col >= 1 && col <= 9) {
            compressed_path_ss << "A" << col << "B" << row;
        } else {
            // 对于中点等特殊情况，直接输出坐标
            compressed_path_ss << "(%.2f,%.2f)" << x << "," << y;
        }
        
        if (i < waypoints.size() - 1) {
            compressed_path_ss << " -> ";
        }
    }
    RCLCPP_INFO(get_logger(), "%s", compressed_path_ss.str().c_str());

}

bool TaskController::tilt_land()
{
    // 静态变量记录降落状态
    static bool stage1_completed = false;  // 第一阶段是否完成
    static bool stage1_started = false;    // 第一阶段是否已开始
    
    // 目标降落位置 (0, 0, 0)
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.0;  // 地面高度
    
    // 获取当前位置
    double current_x = local_position_.pose.position.x;
    double current_y = local_position_.pose.position.y;
    double current_z = local_position_.pose.position.z;
    
    // 计算到目标点(0,0)的水平距离
    double horizontal_distance = sqrt(pow(current_x - target_x, 2) + pow(current_y - target_y, 2));
    
    // 检查是否已经降落完成
    if (current_z < 0.08) {
        RCLCPP_INFO(get_logger(), "降落完成！");
        // 重置状态变量以便下次使用
        stage1_completed = false;
        stage1_started = false;
        return true;
    }
    
    // 45度角降落的理想水平距离应该等于当前高度
    double ideal_horizontal_distance = current_z;  // 45度角条件
    
    // 第一阶段：飞到45度线起点（只执行一次）
    if (!stage1_completed) {
        double position_tolerance = 0.05;  // 位置容差
        
        // 如果还没开始第一阶段，或者距离45度线太远，继续第一阶段
        if (!stage1_started || std::abs(horizontal_distance - ideal_horizontal_distance) > position_tolerance) {
            stage1_started = true;
            
            // 计算45度线起点位置
            double start_point_distance = current_z;  // 45度线起点距离原点的距离等于当前高度
            
            // 计算移动方向
            double move_direction_x, move_direction_y;
            if (horizontal_distance > 0.1) {
                // 沿着当前到原点的方向
                move_direction_x = (target_x - current_x) / horizontal_distance;
                move_direction_y = (target_y - current_y) / horizontal_distance;
            } else {
                // 如果已经在原点正上方，选择一个默认方向
                move_direction_x = 1.0;
                move_direction_y = 0.0;
            }
            
            // 计算45度线起点的目标位置
            double target_start_x = target_x - move_direction_x * start_point_distance;
            double target_start_y = target_y - move_direction_y * start_point_distance;
            
            // 移动到45度线起点（保持当前高度）
            publish_position_setpoint(target_start_x, target_start_y, current_z, 0.0);
            
            RCLCPP_INFO(get_logger(), "第一阶段：移动到45度线起点 [%.2f, %.2f, %.2f] (距离原点: %.2f)",
                        target_start_x, target_start_y, current_z, start_point_distance);
            return false;
        } else {
            // 到达45度线起点，标记第一阶段完成
            stage1_completed = true;
            RCLCPP_INFO(get_logger(), "第一阶段完成！开始准备45度角降落");
        }
    }
    
    
    // 第二阶段：沿45度线降落
    RCLCPP_INFO(get_logger(), "第二阶段：开始45度角降落");
    
    // 计算位置偏差：当前水平距离与理想水平距离的差值
    double horizontal_error = horizontal_distance - ideal_horizontal_distance;
    
    // 固定的垂直下降速度
    double descent_speed = 0.25;  // m/s
    
    // ESP32信号发布
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    esp32_pub_->publish(msg);
    
    // 水平速度控制参数
    double kp_horizontal = 2;  // 比例增益，可根据实际情况调整
    double max_horizontal_speed = 1.0;  // 最大水平速度限制
    
    // 根据水平位置偏差计算水平速度
    double horizontal_speed = 0.0;
    double vel_x_body = 0.0;
    double vel_y_body = 0.0;
    
    if (horizontal_distance > 0.05) {
        // 计算期望的水平速度大小（基于位置偏差）
        horizontal_speed = kp_horizontal * std::abs(horizontal_error);
        
        // 限制最大水平速度
        horizontal_speed = std::min(horizontal_speed, max_horizontal_speed);
        
        // 计算移动方向
        double direction_x = (target_x - current_x) / horizontal_distance;
        double direction_y = (target_y - current_y) / horizontal_distance;
        
        // 如果当前距离大于理想距离，向原点移动（负偏差时也向原点移动）
        // 如果当前距离小于理想距离，远离原点移动
        if (horizontal_error > 0) {
            // 当前距离大于理想距离，需要向原点移动
            direction_x = (target_x - current_x) / horizontal_distance;
            direction_y = (target_y - current_y) / horizontal_distance;
        } else {
            // 当前距离小于理想距离，需要远离原点
            direction_x = (current_x - target_x) / horizontal_distance;
            direction_y = (current_y - target_y) / horizontal_distance;
        }
        
        // 计算世界坐标系下的水平速度
        double vel_x_world = direction_x * horizontal_speed;
        double vel_y_world = direction_y * horizontal_speed;
        
        // 转换到机体坐标系
        vel_x_body = vel_x_world * cos(-current_yaw_) - vel_y_world * sin(-current_yaw_);
        vel_y_body = vel_x_world * sin(-current_yaw_) + vel_y_world * cos(-current_yaw_);
    }
    
    // 发布速度指令（垂直速度始终向下）
    publish_velocity_body(vel_x_body, vel_y_body, -descent_speed, 0.0);
    
    RCLCPP_INFO(get_logger(), 
                "45度角降落控制: 位置[%.2f, %.2f, %.2f] "
                "水平距离: %.2f, 理想距离: %.2f, 偏差: %.2f "
                "速度[%.2f, %.2f, %.2f]",
                current_x, current_y, current_z,
                horizontal_distance, ideal_horizontal_distance, horizontal_error,
                vel_x_body, vel_y_body, -descent_speed);
    
    return false;
}
void TaskController::check_position_stuck_protection()
{
    double current_x = local_position_.pose.position.x;
    double current_y = local_position_.pose.position.y;
    double current_z = local_position_.pose.position.z;
    
    rclcpp::Time current_time = this->get_clock()->now();
    
    // 检查是否在同一位置（使用waypoint_threshold_作为阈值）
    double position_change = std::sqrt(
        std::pow(current_x - last_position_x_, 2) + 
        std::pow(current_y - last_position_y_, 2) + 
        std::pow(current_z - last_position_z_, 2)
    );
    
    if (position_change > waypoint_threshold_) {
        // 位置有显著变化，重置计时器
        last_position_time_ = current_time;
        last_position_x_ = current_x;
        last_position_y_ = current_y;
        last_position_z_ = current_z;
        
        // 如果之前检测到卡住状态，现在位置有变化了，重置状态
        if (position_stuck_detected_) {
            position_stuck_detected_ = false;
            RCLCPP_INFO(get_logger(), "位置恢复移动，重置卡住检测状态");
        }
    } else {
        // 位置变化很小，检查停留时间
        double stuck_duration = (current_time - last_position_time_).seconds();
        
        if (!position_stuck_detected_ && stuck_duration >= 8.0) {
            // 检测到在同一位置停留超过8秒
            position_stuck_detected_ = true;
            stuck_detection_time_ = current_time;
            ignore_target_until_ = current_time + rclcpp::Duration::from_nanoseconds(1000000000LL); // 1秒后
            ignoring_targets_ = true;
            
            RCLCPP_INFO(get_logger(), 
                       "检测到在位置 [%.2f, %.2f, %.2f] 停留超过8秒，开始忽略目标数据1秒",
                       current_x, current_y, current_z);
        }
    }
    
    // 检查是否应该停止忽略目标
    if (ignoring_targets_ && current_time >= ignore_target_until_) {
        ignoring_targets_ = false;
        RCLCPP_INFO(get_logger(), "停止忽略目标数据");
    }
    
    // 打印调试信息（可选）
    if (position_stuck_detected_) {
        double total_stuck_time = (current_time - stuck_detection_time_).seconds();
        double remaining_ignore_time = std::max(0.0, (ignore_target_until_ - current_time).seconds());
        
        RCLCPP_DEBUG(get_logger(), 
                    "位置停留保护状态 - 卡住时间: %.1fs, 剩余忽略时间: %.1fs, 忽略状态: %s",
                    total_stuck_time, remaining_ignore_time, ignoring_targets_ ? "是" : "否");
    }
}
void TaskController::reset_position_stuck_detection()
{
    position_stuck_detected_ = false;
    ignoring_targets_ = false;
    last_position_time_ = this->get_clock()->now();
    last_position_x_ = local_position_.pose.position.x;
    last_position_y_ = local_position_.pose.position.y;
    last_position_z_ = local_position_.pose.position.z;
}

// void TaskController::execute_waypoint_mission()
// {
//     if (current_waypoint_index_ >= waypoints_.size()) {
//         return;
//     }

//     const auto& current_waypoint = waypoints_[current_waypoint_index_];

//     // 检查位置停留保护机制
//     check_position_stuck_protection();

//     // 如果当前有目标并且未处理，且不在忽略目标状态，则优先处理目标
//     if (target_msg_.detected && !ignoring_targets_) {
//         approach();
//         return;  // 等下个 timer_callback 再继续执行航点任务
//     }

//     publish_position_setpoint(current_waypoint[0], current_waypoint[1], current_waypoint[2], current_waypoint[3]);

//     if (is_at_point(current_waypoint[0], current_waypoint[1], current_waypoint[2])) {
//         RCLCPP_INFO(get_logger(), "到达航点 %zu: [%.2f, %.2f, %.2f]",
//                     current_waypoint_index_, current_waypoint[0], current_waypoint[1], current_waypoint[2]);

//         current_waypoint_index_++;
        
//         // 重置位置停留检测
//         reset_position_stuck_detection();
//     }
// }

void TaskController::execute_waypoint_mission()
{
    if (current_waypoint_index_ >= waypoints_.size()) {
        return;
    }
    
    const auto& current_waypoint = waypoints_[current_waypoint_index_];
    
    // 检查位置停留保护机制
    check_position_stuck_protection();
    
    // 如果当前有目标并且未处理，且不在忽略目标状态，则优先处理目标
    if (target_msg_.detected && !ignoring_targets_) {
        approach();
        return;  // 等下个 timer_callback 再继续执行航点任务
    }
    
    // 确定航线的起点
    double start_x, start_y, start_z;
    
    if (current_waypoint_index_ == 0) {
        // 第一个航点，使用当前位置作为起点
        start_x = 0;
        start_y = 0;
        start_z = takeoff_height_;
    } else {
        // 使用上一个航点作为起点
        const auto& previous_waypoint = waypoints_[current_waypoint_index_ - 1];
        start_x = previous_waypoint[0];
        start_y = previous_waypoint[1];
        start_z = previous_waypoint[2];
    }
    
    // 当前航点作为终点
    double end_x = current_waypoint[0];
    double end_y = current_waypoint[1];
    double end_z = current_waypoint[2];
    double target_yaw = current_waypoint[3];
    
    // 使用基于航线的位置控制
    publish_position_setpoint_trajectory(start_x, start_y, start_z,
                                        end_x, end_y, end_z,
                                        target_yaw);
    
    // 检查是否到达当前航点
    if (is_at_point(current_waypoint[0], current_waypoint[1], current_waypoint[2])) {
        RCLCPP_INFO(get_logger(), "到达航点 %zu: [%.2f, %.2f, %.2f]",
                    current_waypoint_index_, current_waypoint[0], current_waypoint[1], current_waypoint[2]);
        current_waypoint_index_++;
        
        // 重置位置停留检测
        reset_position_stuck_detection();
    }
}


    std::string TaskController::world_to_grid(double x, double y) const {
        // 物理格子大小
        constexpr double grid_size = 0.5; // 50cm
        constexpr double judge_half = 0.23; // 40cm判断，中心±0.2m

        // 计算row和col
        int row = static_cast<int>(std::round(x / grid_size)) + 1;  // 1~7
        int col = 9 - static_cast<int>(std::round(y / grid_size));  // 1~9

        // 检查是否在有效范围
        if (row < 1 || row > 7 || col < 1 || col > 9) {
            return "";
        }

        return "A" + std::to_string(col) + "B" + std::to_string(row);
    }

// Helper function to get the next grid cell in the waypoint path
    std::string TaskController::get_next_grid_cell() const {
        if (current_waypoint_index_ >= waypoints_.size()) {
            return "";  // No next waypoint
        }

        const auto& next_waypoint = waypoints_[current_waypoint_index_];
        return world_to_grid(next_waypoint[0], next_waypoint[1]);
    }

// Helper function to check if target is in current or next grid cell
    bool TaskController::is_target_in_valid_grid(double target_world_x, double target_world_y) const {
        // Get current drone position grid
        std::string current_grid = world_to_grid(
                local_position_.pose.position.x,
                local_position_.pose.position.y
        );
        RCLCPP_INFO(get_logger(), "当前网格: %s", current_grid.c_str());

        // Get target position grid
        std::string target_grid = world_to_grid(target_world_x, target_world_y);
        RCLCPP_INFO(get_logger(), "目标网格: %s", target_grid.c_str());

        // Get next waypoint grid
        std::string next_grid = get_next_grid_cell();

        if (current_grid.empty() || target_grid.empty()) {
            RCLCPP_WARN(get_logger(), "无效的网格坐标转换");
            return false;
        }

        // RCLCPP_INFO(get_logger(), "网格检查 - 当前: %s, 目标: %s, 下一个: %s",
        //             current_grid.c_str(), target_grid.c_str(), next_grid.c_str());

        // Check if target is in current grid or next grid
        if (target_grid == current_grid) {
            // RCLCPP_INFO(get_logger(), "目标在当前网格 %s 中", current_grid.c_str());
            return true;
        }

        if (!next_grid.empty() && target_grid == next_grid) {
            // RCLCPP_INFO(get_logger(), "目标在下一个网格 %s 中", next_grid.c_str());
            return true;
        }

        RCLCPP_INFO(get_logger(), "目标不在当前或下一个网格中，忽略目标");
        return false;
    }

void TaskController::target_callback(const msg_tool::msg::Color::ConstSharedPtr& msg) {
    
    if(msg->detected){
        
        // 如果正在忽略目标，直接返回
        if (ignoring_targets_) {
            RCLCPP_DEBUG(get_logger(), "正在忽略目标数据中，跳过目标 %s", msg->color.c_str());
            return;
        }

        constexpr double max_angle_rad = 5 * M_PI / 180.0;
        if(std::abs(current_pitch_) < max_angle_rad && std::abs(current_roll_) < max_angle_rad){
            // 判断当前目标是否已经处理过（颜色 + 坐标）
            double delta_x_world = msg->delta_x * cos(current_yaw_) - msg->delta_y * sin(current_yaw_);
            double delta_y_world = msg->delta_x * sin(current_yaw_) + msg->delta_y * cos(current_yaw_);

            double target_x = local_position_.pose.position.x + delta_x_world;
            double target_y = local_position_.pose.position.y + delta_y_world;

            bool already_processed = false;
            for (const auto &t: processed_targets_) {
                double dist = std::hypot(t.x - target_x, t.y - target_y);
                if (msg->color == t.color && dist < processed_target_radius_) {
                    RCLCPP_INFO(get_logger(), "目标 %s 已经处理过，忽略", msg->color.c_str());
                    already_processed = true;
                    break;
                }
            }

            if (!is_target_in_valid_grid(target_x, target_y)) {

     
                
                return;  // 不处理不在有效网格中的目标
            }

            if (!already_processed) {
                target_msg_ = *msg;
                target_msg_.delta_x = msg->delta_x;
                target_msg_.delta_y = msg->delta_y;
                target_msg_.detected = msg->detected;
            } else {
                // // 发布 processed 标志
                // msg_tool::msg::Color processed_msg;
                // processed_msg.color = msg->color;
                // processed_msg.detected = false;
                // target_pose_pub_->publish(processed_msg);
                
                std_msgs::msg::Bool processed_flag;
                processed_flag.data = true;
                processed_pub_->publish(processed_flag);

            }
        }else{
            RCLCPP_INFO(get_logger(), "无人机姿态不稳定，忽略目标 %s", msg->color.c_str());}
    }
}


bool TaskController::approach() {
    // 世界坐标系下增量
    double delta_x_world = target_msg_.delta_x * cos(current_yaw_) - target_msg_.delta_y * sin(current_yaw_);
    double delta_y_world = target_msg_.delta_x * sin(current_yaw_) + target_msg_.delta_y * cos(current_yaw_);
    double delta_z_world = takeoff_height_ - local_position_.pose.position.z;
    double yaw_diff = 0.0 - current_yaw_;

    // 计算目标靠近位置
    double approach_x = local_position_.pose.position.x + delta_x_world;
    double approach_y = local_position_.pose.position.y + delta_y_world;
    double approach_z = takeoff_height_;
    double approach_yaw = current_yaw_;

    RCLCPP_INFO(get_logger(), "靠近目标: [%.2f, %.2f, %.2f], 当前位置: [%.2f, %.2f, %.2f]",
                approach_x, approach_y, approach_z,
                local_position_.pose.position.x, local_position_.pose.position.y, local_position_.pose.position.z);

    // 使用速度控制靠近目标
    publish_velocity_body(delta_x_world * 1.2, delta_y_world * 1.2, delta_z_world, yaw_diff);

    // 判断是否靠近目标
    if (std::abs(target_msg_.delta_x) < 0.05 && std::abs(target_msg_.delta_y) < 0.05) {
        approach_success_count_++; // 连续帧计数加一
        if (approach_success_count_ >= APPROACH_THRESHOLD) {
            RCLCPP_INFO(get_logger(), "已连续 %d 帧靠近目标，记录位置: [%.2f, %.2f]",
                        APPROACH_THRESHOLD, local_position_.pose.position.x, local_position_.pose.position.y);

            // 记录处理过的目标
            processed_targets_.push_back({target_msg_.color, local_position_.pose.position.x, local_position_.pose.position.y});
            RCLCPP_INFO(get_logger(), "记录目标 color=%s", target_msg_.color.c_str());

            // 发布目标信息
            msg_tool::msg::Color target_info;
            target_info.detected = true;
            target_info.color = target_msg_.color;
            target_info.delta_x = local_position_.pose.position.x;
            target_info.delta_y = local_position_.pose.position.y;
            target_pose_pub_->publish(target_info);

            std_msgs::msg::Bool processed_flag;
            processed_flag.data = true;
            processed_pub_->publish(processed_flag);

            target_msg_.detected = false;
            approach_success_count_ = 0; // 重置计数器
            return true;
        }
    } else {
        approach_success_count_ = 0; // 未满足条件，计数器清零
    }

    target_msg_.detected = false;
    return false;
}



    void TaskController::timer_callback()
    {
        if (!setpoint_ready_) {
            publish_position_setpoint(0.0, 0.0, 0.0, 0.0);
            offboard_setpoint_counter_++;
            if (offboard_setpoint_counter_ > 50) {
                setpoint_ready_ = true;
            }
            return;
        }

        // 检查是否需要切换任务
        check_task_switch_conditions();

        task_state_pub();

        // 执行当前任务
        switch (flight_state_) {
            case FlightState::INIT:
                publish_position_setpoint(
                        local_position_.pose.position.x,
                        local_position_.pose.position.y,
                        local_position_.pose.position.z,
                        0.0);

                // path_pub_->publish(path_msg);

                break;

            case FlightState::TAKEOFF:
                publish_position_setpoint(0.0, 0.0, takeoff_height_, 0.0);
                break;

            case FlightState::WAYPOINT:
                    execute_waypoint_mission();
                break;

            case FlightState::TILTLAND:
                if (tilt_land()) {
                    switch_task(FlightState::LAND);
                }
                break;

            case FlightState::LAND:
                land();
                break;

            default:
                RCLCPP_ERROR(get_logger(), "未知的飞行状态");
                break;
        }
    }
    bool TaskController::check_task_switch_conditions()
    {
        if(check_emergency_condition()){
            RCLCPP_INFO(get_logger(), "检测到紧急情况，自动降落");
            auto_land();
            return true;
        }
        
        switch (flight_state_) {
            case FlightState::INIT:
                if (obstacles_received_){
                    if (launch_flag_) {
                        // 申请进入OFFBOARD模式
                        if (current_state_.mode != "OFFBOARD") {
                            engage_offboard_mode();
                            RCLCPP_INFO(get_logger(), "申请进入OFFBOARD模式");
                            return false;
                        }
                        // 已经是OFFBOARD模式，解锁并切换到TAKEOFF
                        arm();
                        switch_task(FlightState::TAKEOFF);
                        return true;
                    }
                    break;
                }
                
            case FlightState::TAKEOFF:
                if (std::abs(local_position_.pose.position.z - takeoff_height_) < waypoint_threshold_) {
                    switch_task(FlightState::WAYPOINT);
                    return true;
                }
                break;
                
            case FlightState::WAYPOINT:
                // 检查是否所有航点都已完成
                if (current_waypoint_index_ >= waypoints_.size()) {

                    switch_task(FlightState::TILTLAND);
                    return true;
                }
                break;
                
            case FlightState::TILTLAND:
                // TILTLAND状态在timer_callback中处理
                break;
                
            default:
                break;
        }
        return false;
    }

    void TaskController::switch_task(FlightState new_state)
    {
        if (new_state == flight_state_) {
            return;
        }
        RCLCPP_INFO(get_logger(), "切换任务: 从 %d 到 %d",
                    static_cast<int>(flight_state_),
                    static_cast<int>(new_state));
        flight_state_ = new_state;
    }

    bool TaskController::is_at_point(const double x, const double y, const double z) const
    {
        return distance_to_target(x, y, z) < waypoint_threshold_;
    }

    bool TaskController::check_emergency_condition()
    {
        if(std::abs(local_position_.pose.position.x) >= 5 ||
           std::abs(local_position_.pose.position.y) >= 5 ||
           std::abs(local_position_.pose.position.z) >= 2){
            RCLCPP_INFO(get_logger(), "检测到紧急情况，位置超出范围，自动降落");
            return true;
        }
        return false;
    }

} // namespace offboard

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<offboard::TaskController>());
    rclcpp::shutdown();
    return 0;
}