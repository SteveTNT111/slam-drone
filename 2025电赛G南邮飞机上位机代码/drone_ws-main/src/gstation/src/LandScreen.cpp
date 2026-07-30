//
// Created by leon on 25-7-30.
//

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <netinet/in.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <fcntl.h>
#include <msg_tool/msg/color_detections.hpp>
#include<std_msgs/msg/bool.hpp>
// 新增QoS配置
rclcpp::QoS qos_reliable = rclcpp::QoS(10)
    .reliable()
    .durability_volatile();

using json = nlohmann::json;

class LandScreenNode : public rclcpp::Node {
public:
    LandScreenNode() : Node("land_screen") {
        forbidden_publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/nofly_zone", 10);
        // 订阅/target
        animal_target_sub = this->create_subscription<msg_tool::msg::Color>(
            "/target_pose", qos_reliable,
            std::bind(&LandScreenNode::AnimalDetectionsCallback, this, std::placeholders::_1)
        );
        // 订阅/planner_path
        planner_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/planner_path", qos_reliable,
            std::bind(&LandScreenNode::PlannerCallback, this, std::placeholders::_1)
        );
        launch_pub_ = create_publisher<std_msgs::msg::Bool>("/launch", qos_reliable);

        tcp_thread_ = std::thread([this]() { this->tcp_server(); });
        tcp_thread_.detach();

        // 新增定时器，每秒10Hz发送json字符串
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->send_json_to_client(); }
        );

        // 初始化共享数据
        send_data_["tx"] = -1;
        send_data_["ty"] = -1;
        send_data_["tn"] = "NULL";
        send_data_["planner"] = json::array();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr forbidden_publisher_;
    rclcpp::Subscription<msg_tool::msg::Color>::SharedPtr animal_target_sub; // 类型修正
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr planner_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr launch_pub_;
    
    std::thread tcp_thread_;
    rclcpp::TimerBase::SharedPtr send_timer_; // 新增定时器
    std::mutex send_data_mutex;
    json send_data_;

    int last_client_sock_ = -1; // 记录最近的客户端socket

    // 新增回调函数
    void AnimalDetectionsCallback(const msg_tool::msg::Color::SharedPtr msg) {
            // item["a"] = det.delta_x;  // 使用delta_x
            // item["b"] = det.delta_y;  // 使用delta_y
            // item["name"] = det.color;
            std::lock_guard<std::mutex> lock(send_data_mutex);
            if(msg->detected) {
                // 仅在检测到目标时更新数据
                send_data_["tx"] = msg->delta_x;
                send_data_["ty"] = msg->delta_y;
                send_data_["tn"] = msg->color;
            } 


        // RCLCPP_INFO(get_logger(), "Received %zu animal detections", msg->detections.size());
    }

    void PlannerCallback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
        json planner_array = json::array();
        for (const auto& pt : msg->points) {
            json item;
            item["x"] = pt.x;
            item["y"] = pt.y;
            planner_array.push_back(item);
        }
        std::lock_guard<std::mutex> lock(send_data_mutex);
        send_data_["planner"] = planner_array;
        // RCLCPP_INFO(get_logger(), "Received planner path with %zu points", msg->points.size());
    }

    void send_json_to_client() {
        // 仅在有客户端连接时发送
        if (last_client_sock_ < 0) return;
        try {
            json data;
            {
                std::lock_guard<std::mutex> lock(send_data_mutex);
                data = send_data_;
            }
            std::string json_str = data.dump() + "\n";
            ssize_t sent = send(last_client_sock_, json_str.c_str(), json_str.size(), MSG_DONTWAIT);
            (void)sent;
        } catch (...) {}
    }

    void tcp_server() {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8001); // 监听8001端口

        bind(server_fd, (struct sockaddr *)&address, sizeof(address));
        listen(server_fd, 3);

        while (rclcpp::ok()) {
            new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
            if (new_socket < 0) continue;

            // 设置非阻塞
            int flags = fcntl(new_socket, F_GETFL, 0);
            fcntl(new_socket, F_SETFL, flags | O_NONBLOCK);

            {
                // 记录当前客户端socket
                std::lock_guard<std::mutex> lock(send_data_mutex);
                last_client_sock_ = new_socket;
            }

            std::string recv_buffer;
            char temp[4096];
            while (rclcpp::ok()) {
                // 接收部分
                ssize_t received;
                while ((received = recv(new_socket, temp, sizeof(temp), MSG_DONTWAIT)) > 0) {
                    recv_buffer.append(temp, received);
                    // 处理多条JSON（以\n分隔）
                    size_t pos = 0;
                    while ((pos = recv_buffer.find('\n')) != std::string::npos) {
                        std::string message = recv_buffer.substr(0, pos);
                        recv_buffer.erase(0, pos + 1);
                        if (!message.empty()) {
                            try {
                                json j = json::parse(message);
                                // 检查并提取f1x, f1y, f2x, f2y, f3x, f3y
                                if (j.contains("f1x") && j.contains("f1y") &&
                                    j.contains("f2x") && j.contains("f2y") &&
                                    j.contains("f3x") && j.contains("f3y")) {
                                    geometry_msgs::msg::Polygon polygon_msg;

                                    geometry_msgs::msg::Point32 p1, p2, p3;
                                    p1.x = j["f1x"].get<float>();
                                    p1.y = j["f1y"].get<float>();
                                    p1.z = 0.0f;
                                    p2.x = j["f2x"].get<float>();
                                    p2.y = j["f2y"].get<float>();
                                    p2.z = 0.0f;
                                    p3.x = j["f3x"].get<float>();
                                    p3.y = j["f3y"].get<float>();
                                    p3.z = 0.0f;
                                    
                                    polygon_msg.points.push_back(p1);
                                    polygon_msg.points.push_back(p2);
                                    polygon_msg.points.push_back(p3);

                                    if(p1.x>0&&p1.y>0&&p2.x>0&&p2.y>0&&p3.x>0&&p3.y>0) // 过滤异常数据
                                    forbidden_publisher_->publish(polygon_msg);
                                }
                                if(j.contains("launch")) {
                                    std_msgs::msg::Bool launch_msg;
                                    launch_msg.data = j["launch"].get<bool>();
                                    launch_pub_->publish(launch_msg);
                                }
                            } catch (const std::exception&) {
                                // 解析失败，忽略
                            }
                        }
                    }
                }
                if (received == 0) break; // 客户端断开
                else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) break;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            close(new_socket);
            {
                std::lock_guard<std::mutex> lock(send_data_mutex);
                last_client_sock_ = -1;
            }
        }
        close(server_fd);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LandScreenNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
