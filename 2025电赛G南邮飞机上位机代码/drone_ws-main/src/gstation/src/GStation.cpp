#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include <vector>
#include <fcntl.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<std_msgs/msg/int32.hpp>
#include<std_msgs/msg/bool.hpp>
#include<std_msgs/msg/string.hpp>
#include"msg_tool/msg/flight_info.hpp"
#include <arpa/inet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include"msg_tool/msg/color.hpp"
#define ERROR_CODE -1
#define MAX_CLIENTS 5

using json = nlohmann::json;
auto qos_best_effort = rclcpp::QoS(10)
    .best_effort()
    .durability_volatile();

auto qos_reliable = rclcpp::QoS(10)
    .reliable()
    .durability_volatile();

class UAVBridgeNode : public rclcpp::Node {
public:
    UAVBridgeNode() : Node("uav_bridge") {
        // 初始化共享数据
        send_data_ = {
            {"cx", 0.0},
            {"cy", 0.0},
            {"cz", 0.0},
            {"tx", 1.0},
            {"ty", 1.0},
            {"tz", 1.0},
            {"task_id", 1},
            {"state","UNKNOWN"},
            {"armed", false},
            {"mode", "UNKNOWN"},
            {"flash_id",0},
            {"sx",0.0},
            {"sy",0.0},
            {"sz",0.0},
            {"sn","nothing"}
        };
        chosen_target_.color = "nothing";
        chosen_target_.delta_x = 0.0;
        chosen_target_.delta_y = 0.0;
        chosen_target_.detected = false;

        // 创建订阅器
        odom_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_best_effort,
            std::bind(&UAVBridgeNode::odom_callback, this, std::placeholders::_1));
        
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_best_effort,
            std::bind(&UAVBridgeNode::state_callback, this, std::placeholders::_1));

        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", qos_best_effort,
            std::bind(&UAVBridgeNode::target_callback, this, std::placeholders::_1));

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos_best_effort,
            std::bind(&UAVBridgeNode::image_callback, this, std::placeholders::_1));
        task_id_sub_ = create_subscription<msg_tool::msg::FlightInfo>(
            "/task_reply", qos_best_effort,
            [this](const msg_tool::msg::FlightInfo msg) {
                std::lock_guard<std::mutex> lock(send_data_mutex);
                send_data_["task_id"] = msg.task_id;
                send_data_["state"] = msg.state;
            });
        sought_target_sub = create_subscription<msg_tool::msg::Color>(
            "/target_pose",qos_best_effort,[this](const msg_tool::msg::Color msg)
            {
                std::lock_guard<std::mutex> lock(send_data_mutex);
                send_data_["sx"] = msg.delta_x;
                send_data_["sy"] = msg.delta_y;
                send_data_["sn"] = msg.color;
            });
        task_id_pub_ = create_publisher<std_msgs::msg::Int32>("/task", qos_best_effort);
        flash_id_sub_ = create_subscription<std_msgs::msg::Int32>("/flash_id", qos_reliable,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(send_data_mutex);
            send_data_["flash_id"] = msg->data; 
        });
        chosen_target_pub_ = create_publisher<msg_tool::msg::Color>("/go_target", qos_best_effort);
        // 启动TCP服务器线程
        json_server_thread_ = std::thread(&UAVBridgeNode::json_tcp_server, this);
        image_server_thread_ = std::thread(&UAVBridgeNode::image_udp_server, this);
        task_id_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->task_id_timer_callback(); });
        task_id_.data = 1;
        RCLCPP_INFO(get_logger(), "UAV Bridge Node started");
    }

    ~UAVBridgeNode() {
        running_ = false;
        if (json_server_thread_.joinable()) {
            json_server_thread_.join();
        }
        if (image_server_thread_.joinable()) {
            image_server_thread_.join();
        }
    }

private:
    // 回调函数：处理里程计数据


    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(send_data_mutex);
        send_data_["cx"] = msg->pose.position.x;
        send_data_["cy"] = msg->pose.position.y;
        send_data_["cz"] = msg->pose.position.z;
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        send_data_["cyaw"] = yaw;
    }

    // 回调函数：处理无人机状态
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(send_data_mutex);
        send_data_["armed"] = msg->armed;
        send_data_["mode"] = msg->mode;
    }

    // 回调函数：处理目标位置
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(send_data_mutex);
        send_data_["tx"] = msg->pose.position.x;
        send_data_["ty"] = msg->pose.position.y;
        send_data_["tz"] = msg->pose.position.z;
    }
    // JSON数据TCP服务器
    void json_tcp_server() {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == ERROR_CODE) {
            RCLCPP_ERROR(get_logger(), "JSON Socket creation failed");
            return;
        }

        // 设置套接字选项
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            RCLCPP_ERROR(get_logger(), "JSON Setsockopt failed");
            close(server_fd);
            return;
        }

        // 绑定地址和端口
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8000);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == ERROR_CODE) {
            RCLCPP_ERROR(get_logger(), "JSON Bind failed");
            close(server_fd);
            return;
        }

        if (listen(server_fd, MAX_CLIENTS) == ERROR_CODE) {
            RCLCPP_ERROR(get_logger(), "JSON Listen failed");
            close(server_fd);
            return;
        }

        RCLCPP_INFO(get_logger(), "JSON TCP server listening on port 8000");

        std::vector<std::thread> client_threads;

        running_ = true;
        while (running_ && rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);

            int client_sock = accept(server_fd, (struct sockaddr*)&client_addr, &addr_len);
            if (client_sock < 0) {
                if (running_) RCLCPP_ERROR(get_logger(), "JSON Accept failed");
                continue;
            }

            // 设置非阻塞模式
            int flags = fcntl(client_sock, F_GETFL, 0);
            fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

            // 为每个客户端创建独立线程
            client_threads.emplace_back([this, client_sock]() {
                this->handle_json_client(client_sock);
            });
        }

        // 清理线程
        for (auto& t : client_threads) {
            if (t.joinable()) t.join();
        }

        close(server_fd);
    }

    // 处理JSON客户端连接
    void handle_json_client(int client_sock) {
        RCLCPP_INFO(get_logger(), "New JSON client connected");
    
        std::string recv_buffer;  // 接收缓冲区，用于拼接不完整的 JSON
        char temp[4096];          // 临时接收数据的 buffer
    
        while (running_ && rclcpp::ok()) {
            // --- 发送部分：将 send_data_ 发送给客户端 ---
            try {
                nlohmann::json data;
                {
                    std::lock_guard<std::mutex> lock(send_data_mutex);
                    data = send_data_;
                }
    
                std::string json_str = data.dump() + "\n";  // 加换行便于分隔
                ssize_t sent = send(client_sock, json_str.c_str(), json_str.size(), MSG_DONTWAIT);
                send_data_["flash_id"] = 0; // 重置 flash_id 以避免重复发送
                if (sent == ERROR_CODE) {
                    if (errno != EWOULDBLOCK && errno != EAGAIN) {
                        RCLCPP_ERROR(get_logger(), "JSON Send error: %s", strerror(errno));
                        break;
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error during sending JSON: %s", e.what());
            }
    
            // --- 接收部分：尝试从客户端读取 JSON 消息 ---
            ssize_t received;
            while ((received = recv(client_sock, temp, sizeof(temp), MSG_DONTWAIT)) > 0) {
                recv_buffer.append(temp, received);
    
                // 尝试解析 JSON
                try {
                    size_t pos = 0;
                    while ((pos = recv_buffer.find('\n')) != std::string::npos) {
                        std::string message = recv_buffer.substr(0, pos);
                        recv_buffer.erase(0, pos + 1);
    
                        if (!message.empty()) {
                            try {
                                nlohmann::json json_msg = nlohmann::json::parse(message);
                                // RCLCPP_INFO(get_logger(), "Received from client: %s", json_msg.dump().c_str());
    
                                // 在这里可以调用回调或更新状态
                                handle_received_json(json_msg);  // 自定义处理函数
                            } catch (const nlohmann::json::parse_error&) {
                                RCLCPP_WARN(get_logger(), "Failed to parse JSON: %s", message.c_str());
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Error parsing client message: %s", e.what());
                }
            }
    
            if (received == 0) {
                RCLCPP_INFO(get_logger(), "Client disconnected gracefully");
                break;
            } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(get_logger(), "JSON recv error: %d (%s)", errno, strerror(errno));
                break;
            }
    
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 避免 CPU 占满
        }
    
        close(client_sock);
        RCLCPP_INFO(get_logger(), "JSON client disconnected");
    }
// 图像UDP服务器
void image_udp_server() {
    int server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_fd == ERROR_CODE) {
        RCLCPP_ERROR(get_logger(), "Image UDP Socket creation failed");
        return;
    }

    // 设置套接字选项
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        RCLCPP_ERROR(get_logger(), "Image UDP Setsockopt failed");
        close(server_fd);
        return;
    }

    // 绑定地址和端口
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(9002);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) == ERROR_CODE) {
        RCLCPP_ERROR(get_logger(), "Image UDP Bind failed");
        close(server_fd);
        return;
    }

    // 创建目标地址 - 固定为 127.0.0.1:9001
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(9001);
    inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr);

    // 使用 connect 创建已连接的 UDP 套接字
    if (connect(server_fd, (struct sockaddr*)&dest_addr, sizeof(dest_addr)) == ERROR_CODE) {
        RCLCPP_ERROR(get_logger(), "Image UDP Connect failed: %s", strerror(errno));
        close(server_fd);
        return;
    }

    RCLCPP_INFO(get_logger(), "Image UDP server connected to 127.0.0.1:9001");

    running_ = true;
    while (running_ && rclcpp::ok()) {
        send_image_data(server_fd);
        
        // 控制发送频率
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 约30fps
    }

    close(server_fd);
    RCLCPP_INFO(get_logger(), "Image UDP server stopped");
}

// 发送图像数据（包含头信息和图像数据在同一个UDP包中）
void send_image_data(int sock_fd) {
    std::shared_ptr<CompressedImage> img;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        img = compressed_image_;
    }
    
    if (img && !img->data.empty()) {
        // 创建包含头部和图像数据的完整UDP数据包
        std::vector<uint8_t> udp_packet;
        // 预分配足够的空间
        udp_packet.resize(16 + img->data.size());
        uint32_t img_size = static_cast<uint32_t>(img->data.size());
        // 填充头部信息
        memcpy(udp_packet.data(), &img->width, 4);
        memcpy(udp_packet.data() + 4, &img->height, 4);
        memcpy(udp_packet.data() + 8, &img->channels, 4);
        memcpy(udp_packet.data() + 12, &img_size, 4);
        
        // 填充图像数据
        memcpy(udp_packet.data() + 16, img->data.data(), img->data.size());
        
        // 发送完整的UDP数据包 - 使用已连接的套接字，直接用 send 而非 sendto
        ssize_t sent = send(sock_fd, udp_packet.data(), udp_packet.size(), 0);
        
        if (sent == ERROR_CODE) {
            // RCLCPP_ERROR(get_logger(), "Image UDP send error: %s", strerror(errno));
        } else if (sent != static_cast<ssize_t>(udp_packet.size())) {
            // RCLCPP_WARN(get_logger(), "Partial UDP packet sent: %zd/%zu bytes", 
                    //    sent, udp_packet.size());
        }
    }
}
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // RCLCPP_INFO(this->get_logger(),"image callback");
            // 使用cv_bridge转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    
            // 裁剪到 640x480
            cv::Mat resized_img;
            cv::resize(cv_ptr->image, resized_img, cv::Size(640, 480));
    
            // 压缩图像为JPEG格式（质量80%，可调整）
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(80);
    
            std::vector<uchar> jpeg_buffer;
            cv::imencode(".jpg", resized_img, jpeg_buffer, params);
    
            // 更新共享数据
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                compressed_image_ = std::make_shared<CompressedImage>();
                compressed_image_->data = std::move(jpeg_buffer);
                compressed_image_->width = 640;
                compressed_image_->height = 480;
                compressed_image_->channels = resized_img.channels();
            }
    
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    void handle_received_json(const nlohmann::json& json_msg) {
        std::lock_guard<std::mutex> lock(receive_data_mutex);   
        if(json_msg.contains("task"))  task_id_.data = json_msg["task"].get<int>();
        if(json_msg.contains("launch")) launch_flag_.data = json_msg["launch"].get<bool>();
        if (json_msg.contains("sx")) chosen_target_.delta_x = json_msg["sx"].get<double>();
        if (json_msg.contains("sy")) chosen_target_.delta_y = json_msg["sy"].get<double>();
        if (json_msg.contains("sn"))chosen_target_.color = json_msg["sn"].get<std::string>();
        // RCLCPP_INFO(get_logger(), "Received task ID: %d", task_id_.data);
    }

    void task_id_timer_callback()
    {
        task_id_pub_->publish(task_id_);
        if (chosen_target_.color != "nothing")
        {
            chosen_target_.detected = true;
            chosen_target_pub_->publish(chosen_target_);
            // RCLCPP_INFO(get_logger(), "Chosen target published");
        }

    }
    // 自定义压缩图像结构
    struct CompressedImage {
        std::vector<uchar> data;
        uint32_t width;
        uint32_t height;
        uint32_t channels;
    };
    void handle_image_client(int client_sock) {
        RCLCPP_INFO(get_logger(), "New image client connected");
    
        // 预分配头缓冲区
        uint8_t header[16]; // 12字节图像头 + 4字节数据长度
    
        while (running_ && rclcpp::ok()) {
            std::shared_ptr<CompressedImage> img;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                img = compressed_image_;
            }
    
            if (img && !img->data.empty()) {
                // 构建头信息
                uint32_t net_width = htonl(img->width);
                uint32_t net_height = htonl(img->height);
                uint32_t net_channels = htonl(img->channels);
                uint32_t net_img_size = htonl(static_cast<uint32_t>(img->data.size()));
    
                memcpy(header, &net_width, 4);
                memcpy(header + 4, &net_height, 4);
                memcpy(header + 8, &net_channels, 4);
                memcpy(header + 12, &net_img_size, 4);
    
                // 发送头信息
                ssize_t sent = send(client_sock, header, 16, MSG_DONTWAIT);
                if (sent == ERROR_CODE) {
                    if (errno != EWOULDBLOCK && errno != EAGAIN) {
                        RCLCPP_ERROR(get_logger(), "Image header send error: %s", strerror(errno));
                        break;
                    }
                }
    
                // 发送图像数据
                const uint8_t* data_ptr = img->data.data();
                size_t total_sent = 0;
                size_t total_size = img->data.size();
    
                while (total_sent < total_size) {
                    ssize_t n = send(client_sock, data_ptr + total_sent, 
                                    total_size - total_sent, MSG_DONTWAIT);
    
                    if (n > 0) {
                        total_sent += n;
                    } else if (n == ERROR_CODE) {
                        if (errno != EWOULDBLOCK && errno != EAGAIN) {
                            RCLCPP_ERROR(get_logger(), "Image data send error: %s", strerror(errno));
                            break;
                        }
                        // 避免在非阻塞模式下过度占用CPU
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            }
    
            // 控制发送频率
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 约30fps
        }
    
        close(client_sock);
        RCLCPP_INFO(get_logger(), "Image client disconnected");
    }
    // ROS订阅器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_id_pub_;
    rclcpp::Subscription<msg_tool::msg::FlightInfo>::SharedPtr task_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flash_id_sub_;
    rclcpp::Subscription<msg_tool::msg::Color>::SharedPtr sought_target_sub;
    rclcpp::Publisher<msg_tool::msg::Color>::SharedPtr chosen_target_pub_;
    rclcpp::TimerBase::SharedPtr task_id_timer_;
    // 线程和同步
    std::thread json_server_thread_;
    std::thread image_server_thread_;
    std::mutex send_data_mutex;
    std::mutex receive_data_mutex;
    std::mutex image_mutex_;

    // 共享数据
    json send_data_;
    std::shared_ptr<CompressedImage> compressed_image_;
    std::atomic<bool> running_{true};
    std_msgs::msg::Int32 task_id_; // 任务ID
    std_msgs::msg::Bool launch_flag_; // 启动标志
    msg_tool::msg::Color chosen_target_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 忽略SIGPIPE信号（防止写入断开连接时崩溃）
    signal(SIGPIPE, SIG_IGN);

    auto node = std::make_shared<UAVBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}