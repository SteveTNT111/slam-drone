// 8/5 7:15
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Int8.h> 
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>
#include <stdlib.h>

#define TargeyHeight 1.75
#define TargetYaw 0.2
#define TargetRadius 0.5
#define Limitv_x 0.5
#define Limitv_y 0.5
#define Limitv_z 0.5
#define Limit_yaw 0.5

ros::Subscriber state_sub;//飞机当前状态cur
ros::Subscriber pose_sub;//飞机当前位置信息
ros::Subscriber vel_sub;//飞机速度信息
ros::Subscriber fire_pose_sub;
ros::Publisher local_pose_pub;//设置飞机目标位置（全局坐标
ros::Publisher vel_pub;//设置飞机速度（全局坐标
ros::Publisher state_pub;//飞机状态
ros::Publisher set_raw_pub; //以飞机自己为坐标系的速度控制
ros::Publisher hit_pub;
ros::Publisher drop_pub;
ros::Publisher state_message_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
geometry_msgs::TwistStamped vel_cmd; //全局消息，速度设置
geometry_msgs::PoseStamped pose;//全局消息，分别用来控制飞机自身的位置与飞机降落add_executable(pole_test src/pole_test.cpp)
int land_distance = -1;
double last_error_yaw=0, last_error_dist=0;
double yaw_i = 0 , dist_i = 0;
bool cur_state;
int fire_flag = 0;

bool state_cb(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){  //用于通过命令启动飞机
    cur_state=!cur_state;
    // 显示请求数据
    ROS_INFO("Publish command [%s]", cur_state==true?"Yes":"No");
	// 设置反馈数据
	res.success = true;
	res.message = "Change drone command state!";
    return true;
}

double min(double a,double b){   //比较ab较小的一个数，其中a的值是被比较值，b是限制量（注意，不可用反
    double ret=0;
    if (a == 0){
        ret=0;
    }else if(fabs(a)>=fabs(b)){
        ret=fabs(b)*a/fabs(a);
    }else{
        ret=fabs(a)*a/fabs(a);
    }
    return ret;
}

double arc_normalize_yaw(double current_yaw){
    double raw_yaw=current_yaw;
    if(fabs(current_yaw)>3.14){
        raw_yaw = current_yaw > 0 ? current_yaw - 6.28319 : current_yaw + 6.28319;
    }
    return raw_yaw;
}

mavros_msgs::State current_state;  //订阅消息：当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose; //订阅消息：当前位置（世界坐标系）
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

geometry_msgs::TwistStamped current_vel; //订阅消息：当前速度
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //ROS_INFO("Current velocity: %.2f m/s", msg->twist.linear.x);
    current_vel = *msg;
}


std_msgs::ColorRGBA fire_pose;
void fire_pose_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    fire_pose = *msg;
}

// std_msgs::Float64MultiArray pole_circle_pose;
// void fire_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
// {
//     pole_circle_pose = *msg;
// }


std_msgs::ColorRGBA error_to_target;
void land_node_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    error_to_target = *msg;
}


class move_map
{
private:
    double kpx,kpy,kpz;
    double kix,kiy,kiz;
    double kdx,kdy,kdz;
    double range,base;
    double kp;
    double ki;
    double kd;
    double last_x, last_y, last_z;
    double last_vx, last_vy, last_vz;
    double ix, iy ,iz; 
    double acceleration_limit;

    double radius = 1.0;//半径
    double speed = 0.2;
    double angular_speed = speed / radius;
    double angle = 0.0;
    double rotation_angle;
    double last_angle = 0;

    int interupt_step = 15;
    double height = TargeyHeight;
    geometry_msgs::PoseStamped interupt_pose;

    double func(double error,double A,double B){ //变速积分的子函数，主要就是对于积分的判断
    double result;
    if(error>=B+A){
        result = 0;
    }else if(error<B+A && error>B){
        result = (A+B-error)/A;
    }else{
        result = 1;
    }
    return result;
    }

public:
    int fly_to_target(geometry_msgs::PoseStamped target_pose,ros::Publisher& local_vel_pub,
ros::Rate& rate) {
    int judge=0;
    // 初始化PID控制器
        double ex = 0.0, ey = 0.0, ez = 0.0;     
        // 计算PID控制器输出
        double dt = rate.expectedCycleTime().toSec();
        ex = target_pose.pose.position.x - current_pose.pose.position.x;
        ey = target_pose.pose.position.y - current_pose.pose.position.y;
        ez = target_pose.pose.position.z - current_pose.pose.position.z;
        ix += ex * dt * func(ex,range,base);
        iy += ey * dt * func(ey,range,base);
        iz += ez * dt * func(ez,range,base);
        ix = min(ix,0.3);
        iy = min(iy,0.3);
        iz = min(iz,0.3);
        double dx = (ex - last_x) / dt;
        double dy = (ey - last_y) / dt;
        double dz = (ez - last_z) / dt;
        double vx = kp * ex + ki * ix + kd * dx;
        double vy = kp * ey + ki * iy + kd * dy;
        double vz = kpz * ez + kiz * iz + kdz * dz;
        ROS_INFO("vx = %.2lf vy = %.2lf vz = %.2lf",vx,vy,vz);
        last_x = ex;
        last_y = ey;
        last_z = ez;
        // 更新速度指令
        geometry_msgs::TwistStamped vel;
        vel.header.stamp = ros::Time::now();
        vel.twist.linear.x = min(vx,Limitv_x);
        vel.twist.linear.y = min(vy,Limitv_y);
        vel.twist.linear.z = min(vz,Limitv_z);
        vel.twist.angular.x = 0.0;
        vel.twist.angular.y = 0.0;
        vel.twist.angular.z = 0.0;
        // 发布速度指令 
        local_vel_pub.publish(vel);
        // 判断是否到达目标点
        if (fabs(ex) < 0.1 && fabs(ey) < 0.1 ) {
            judge = 1;
        }
        return judge;
}

int circle_locator(std_msgs::ColorRGBA RGB,ros::Publisher& local_vel_pub,ros::Rate& rate){  //对准圆心函数
    int judge = 0;
    double err_x = -RGB.g/300; 
    double err_y = -RGB.r/300;
    ROS_INFO("err = %.2lf,err = %.2lf", err_x, err_y);
    double dt = rate.expectedCycleTime().toSec();
    double ez = height - current_pose.pose.position.z;
    ix += err_x * dt * func(err_x,range,base);
    iy += err_y * dt * func(err_y,range,base);
    iz += ez * dt * func(ez,range,base);
    ix = min(ix,0.1);
    iy = min(iy,0.1);
    iz = min(iz,0.1);
    double dx = (err_x - last_x) / dt;
    double dy = (err_y - last_y) / dt;
    double dz = (ez - last_z) / dt;
    double vx = kp * err_x + ki * ix + kd * dx;
    double vy = kp * err_y + ki * iy + kd * dy;
    double vz = kpz * ez + kiz * iz + kdz * dz;
    geometry_msgs::TwistStamped vel;
    vel.header.stamp = ros::Time::now();
    vel.twist.linear.x = min(vx,Limitv_x);
    vel.twist.linear.y = min(vy,Limitv_y);
    vel.twist.linear.z = min(vz,Limitv_z);
    ROS_INFO("vx = %.2lf,vy = %.2lf,vz = %.2lf", vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z);
    vel.twist.angular.x = 0.0;
    vel.twist.angular.y = 0.0;
    vel.twist.angular.z = 0.0;
    // 发布速度指令 
    local_vel_pub.publish(vel);
    // 判断是否到达目标点
    return judge;
}

    void hold(ros::Publisher& local_vel_pub){
    geometry_msgs::TwistStamped vel;
    vel.header.stamp = ros::Time::now();
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0;
    local_vel_pub.publish(vel);
    }

    int pole_rotation(geometry_msgs::PoseStamped center_pose,ros::Publisher& local_vel_pub,
ros::Rate& rate){
    int flag = 0;
    double dt = rate.expectedCycleTime().toSec();
    angle += dt * angular_speed;
    //ROS_INFO("target_angle = %.2lf",angle);
    double px = radius * cos(angle)+center_pose.pose.position.x;
    double py = radius * sin(angle)+center_pose.pose.position.y;
    if(sqrt((px - current_pose.pose.position.x)*(px - current_pose.pose.position.x)+
    (py - current_pose.pose.position.y)*(py - current_pose.pose.position.y)
    )>0.3){
        angle -= dt * angular_speed * 3;
        //ROS_INFO("angle too large tar_angle = %.2lf",angle);
        pose.pose.position.x = radius * cos(last_angle)+center_pose.pose.position.x;
        pose.pose.position.y = radius * sin(last_angle)+center_pose.pose.position.y;
        pose.pose.position.z = TargeyHeight;
        local_pose_pub.publish(pose);
    }else{
    pose.pose.position.x = radius * cos(angle)+center_pose.pose.position.x;
    pose.pose.position.y = radius * sin(angle)+center_pose.pose.position.y;
    pose.pose.position.z = TargeyHeight;
    last_angle = angle;
    local_pose_pub.publish(pose);
    }        
    if(fabs(angle - rotation_angle) < 0.05){
        ROS_INFO("angle = %.2lf , current_angle = %.2lf",angle,rotation_angle);
        flag = 1;
    }
    return flag;
}

    int fire_global_interupt(int step){
       if(fire_pose.a == 1 && fire_flag==0){
            last_x = 0.0, last_y = 0.0, last_z = 0.0;
            ix = 0.0, iy = 0.0, iz = 0.0; 
            last_vx = 0.0, last_vy = 0.0, last_vz = 0.0;
            interupt_step = step;
            interupt_pose = current_pose;
            step = 100;
            ROS_INFO("interupt %d",step);
            return step;
       }else{
            return step;
       }
    }

    void set_pid_all(double p,double i,double d){
        kp = p;
        ki = i;
        kd = d;
    }

    void set_pid_z(double p,double i,double d){
        kpz = p;
        kiz = i;
        kdz = d;
    }



    void pid_init(){
        last_x = 0.0, last_y = 0.0, last_z = 0.0;
        ix = 0.0, iy = 0.0, iz = 0.0; 
        last_vx = 0.0, last_vy = 0.0, last_vz = 0.0;
    }

    void set_acceleration_limit(double a){
        acceleration_limit = a;
    }

    void printpose(){
        ROS_INFO("px = %.2lf,py = %.2lf,pz = %.2lf",
        current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    }

    void printpid(){
        ROS_INFO("ix = %.2lf,iy = %.2lf,iz = %.2lf",
        ix,iy,iz);
    }    

    void set_height(double height_input){
        height = height_input;
    }

    int get_interupt_step(){
        return interupt_step;
    }

    geometry_msgs::PoseStamped get_interupt_pose(){
        return interupt_pose;
    }


    void circle_init(double r,double linear_vel,double current_angle,double spin_angle){
        radius = r;//半径
        speed = linear_vel;
        angular_speed = speed / radius;
        angle = current_angle;
        rotation_angle = spin_angle*min(linear_vel,0.1)*10 + current_angle;
        last_angle = current_angle;
        ROS_INFO("circle init: r= %.2lf,current_angle = %.2lf",
        r,current_angle);
    }
};


int main(int argc, char **argv)
{
    int step=1;
    double target_speed=0.5;// 初始化 ROS 节点
    int rotation_direction=0;
    std_msgs::ColorRGBA lidar_data_sender;
    double* pole_data;
    double yaw_angle,start_yaw;
    std_msgs::Int8 vision_cmd_sender;
    std_msgs::Int16 led_data_sender;
    led_data_sender.data = 0;
    vision_cmd_sender.data = 1;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // state_sub = nh.subscribe<mavros_msgs::State>
    //     ("/iris_0/mavros/state", 10, state_cb);  //订阅状态
    // pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("/iris_0/mavros/local_position/pose", 10, pose_cb); //订阅位置信息.
    // vel_sub = nh.subscribe<geometry_msgs::TwistStamped> 
    //     ("/iris_0/mavros/local_position/velocity", 10 ,vel_cb);
    // fire_pose_sub = nh.subscribe<std_msgs::ColorRGBA>//发送雷达节点数据，用来规定发送的角度/vision/fire
    //     ("/vision/fire",10,fire_pose_cb);
    // local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //     ("/iris_0/mavros/setpoint_position/local", 10);  //设置位置
    // vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //     ("/iris_0/mavros/setpoint_velocity/cmd_vel", 10);  //设定速度
    // set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
    //     ("/iris_0/mavros/setpoint_raw/local", 10);
    // drop_pub = nh.advertise<std_msgs::Int8>
    //     ("/offboard/drop",10);
    //  state_message_pub = nh.advertise<std_msgs::Int8>
    //      ("/offboard_node/state",10);   
    // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //     ("/iris_0/mavros/cmd/arming");   
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //     ("/iris_0/mavros/set_mode");
    // ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
    //     ("/iris_0/mavros/cmd/land");
    // ros::ServiceServer state_client = nh.advertiseService
    //     ("/command",state_cb);          

    
    state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, state_cb);  //订阅状态
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, pose_cb); //订阅位置信息.
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped> 
        ("/mavros/local_position/velocity", 10 ,vel_cb);
    fire_pose_sub = nh.subscribe<std_msgs::ColorRGBA>//发送雷达节点数据，用来规定发送的角度/vision/fire
        ("/vision/fire",10,fire_pose_cb);
    local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);  //设置位置
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);  //设定速度
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 10);
    drop_pub = nh.advertise<std_msgs::Int8>
       ("/offboard/drop",10);
    state_message_pub = nh.advertise<std_msgs::Int8>
        ("/offboard_node/state",10);
    // 定义服务客户端，用于解锁/上锁无人机和切换到离线控制模式
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");   
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("/mavros/cmd/land");
    ros::ServiceServer state_client = nh.advertiseService
        ("/command",state_cb);    

    // 指定发布位置设定点的频率
    ros::Rate rate(50.0);  
    // 等待 FCU 连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 发布一些起始位置设定点，然后才开始执行控制指令
    // 请求进入离线控制模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // 请求解锁飞行器
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    // 记录上一次请求时间
    ros::Time last_request = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    ros::Time fire_time = ros::Time::now();
    mavros_msgs::CommandTOL land_cmd;
    std_msgs::Int8 offboard_state;
    std_msgs::Int8 drop_command;
    drop_command.data = 0;
    geometry_msgs::PoseStamped target_position;
    geometry_msgs::PoseStamped landing_position;
    geometry_msgs::PoseStamped rounding_pose[3];
    double *target_position_ptr;
    land_cmd.request.altitude = 0.0;
    land_cmd.request.yaw = 0.0;
    int flag = 0;
    double radius = 1.0;
    double speed = 0.5;
    double angular_speed = speed / radius;
    double angle = 0.0;
    move_map move1;
    while(ros::ok()){
        ros::spinOnce();
        //ROS_INFO("%.2lf",fire_pose.a);
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_WARN("Offboard enabled");
                //cur_state = 1;
                target_position = current_pose;
                landing_position = current_pose;
                target_position.pose.position.z = TargeyHeight;
                move1.pid_init();
                move1.set_pid_z(0.7,0.1,0.1);
                move1.set_pid_all(0.8,0.2,0.2);
                move1.set_pid_z(0.8,0.1,0.1);
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    ROS_INFO("step1");
                }
                last_request = ros::Time::now();
            }
            if(cur_state){
                switch(step){
            case 1:  // case1 起飞程序
                if(current_pose.pose.position.z < (target_position.pose.position.z - 0.12)){
                    //move1.fly_to_target(target_position,vel_pub,rate);
                    //move1.printpose();
                    local_pose_pub.publish(target_position);
                }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    move1.set_height(TargeyHeight);
                    target_position.pose.position.x = 0+0.22;
                    target_position.pose.position.y = 3.1-0.2;
                    ROS_INFO("step2");
                    step = 2;
                    //step = 100;
                    flag = 0;
                }
                break;    
            case 2: 
            if(flag == 0){
                //step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 4-0.13;//第二个点的位置
                    target_position.pose.position.y = 3.1;
                    ROS_INFO("step3");
                    step = 3;
                    flag = 0;
                }
            break;
            case 3: 
            if(flag == 0){
                //step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 4-0.13;//第三个点的位置
                    target_position.pose.position.y = 0 + 0.23;
                    ROS_INFO("step4");
                    step = 4;
                    flag = 0;
                }
            break;
             case 4: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 3.35;
                    target_position.pose.position.y = 0+0.23;
                    ROS_INFO("step5");
                    step = 5;
                    flag = 0;
                }
            break;
            case 5: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose35
                    target_position.pose.position.x = 3.35;
                    target_position.pose.position.y = 2.3;
                    ROS_INFO("step6");
                    step = 6;
                    flag = 0;
                }
            break;
            case 6: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 2.4;
                    target_position.pose.position.y = 2.3;
                    ROS_INFO("step7");
                    step = 7;
                    flag = 0;
                }
            break;
            case 7: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 2.4;
                    target_position.pose.position.y = 0+0.23;
                    ROS_INFO("step8");
                    step = 8;
                    flag = 0;
                }
            break;
            case 8: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 1.6;
                    target_position.pose.position.y = 0+0.23;
                    ROS_INFO("step9");
                    step = 9;
                    flag = 0;
                }
            break;
            case 9: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 1.6;
                    target_position.pose.position.y = 2.3;
                    ROS_INFO("step10");
                    step = 10;
                    flag = 0;
                }
            break;
            case 10: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 0.8;
                    target_position.pose.position.y = 2.3;
                    ROS_INFO("step11");
                    step = 11;
                    flag = 0;
                }
              break;
            case 11: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = 0.8;
                    target_position.pose.position.y = 0;
                    ROS_INFO("step12");
                    step = 12;
                    flag = 0;
                }
            break;
            case 12: 
            if(flag == 0){
                step = move1.fire_global_interupt(step);
                fire_time = ros::Time::now();
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    // target_position = current_pose;
                    target_position.pose.position.x = landing_position.pose.position.x;
                    target_position.pose.position.y = landing_position.pose.position.y;
                    ROS_INFO("step13");
                    step = 13;
                    flag = 0;
                }
                break;
            case 13: 
            if(flag == 0){
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0.1,0.2);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    target_position = current_pose;
                    target_position.pose.position.x = landing_position.pose.position.x;
                    target_position.pose.position.y = landing_position.pose.position.y;
                    target_position.pose.position.z = 0;
                    ROS_INFO("step14");
                    step = 14;
                    flag = 0;
                }
                break;
            case 14://返航
                if(ros::Time::now() - last_time < ros::Duration(5.0)){
                    local_pose_pub.publish(target_position);
                }else{
                    last_time = ros::Time::now();
                    ROS_INFO("step15");
                    step = 15;
                    landing_position.pose.position.z = -0.2;
                }
            break;      
            case 15:
            if(current_pose.pose.position.z<0.05){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                    return 0;
                    }
             }else{
                    move1.fly_to_target(landing_position,vel_pub,rate);
                    //move1.velocity_land(rate);
                }
            break;
            case 100:
            if(fire_pose.a == 1){
                if(
                    fabs(fire_pose.r) > 20 || fabs(fire_pose.g) > 20
                    ){
                    move1.set_pid_all(0.8,0,0);
                    move1.circle_locator(fire_pose,vel_pub,rate);
                    target_position = current_pose;
                }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0,0);
                    move1.set_height(0.98);
                    fire_time = ros::Time::now(); //等待接收杆的数据
                    target_position = current_pose;
                    ROS_INFO("step101");
                    step = 101;
                    flag = 0;
                }
            }else{
                move1.hold(vel_pub);
                //local_pose_pub.publish(target_position);
                }
            break;
            case 101:
            if(fire_pose.a == 1){
                if(fabs(fire_pose.r) > 5 || fabs(fire_pose.g) > 5){
                    move1.circle_locator(fire_pose,vel_pub,rate);
                    target_position = current_pose;
                }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0,0);
                    move1.set_height(0.9);
                    fire_time = ros::Time::now(); //等待接收杆的数据
                    target_position = current_pose;
                    ROS_INFO("step102");
                    step = 102;
                    flag = 0;
                }
            }else{
                move1.hold(vel_pub);
                local_pose_pub.publish(target_position);
                }
            break;
            case 102:
            if(ros::Time::now() - fire_time < ros::Duration(6.0)){
                local_pose_pub.publish(target_position);
                if(ros::Time::now() - fire_time > ros::Duration(2.0)){
                    drop_command.data = 1;
                    drop_pub.publish(drop_command);
                    ROS_INFO("drop!");
                }
            }else{
                    move1.pid_init();
                    move1.set_pid_all(0.8,0,0);
                    move1.set_pid_z(1.2,0.2,0.1);
                    move1.set_height(TargeyHeight);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    rounding_pose[0] = current_pose;
                    target_position = current_pose;
                    target_position.pose.position.x -=0.8;
                    step = 103;
                    ROS_INFO("step =  %d",step);
                    flag = 0;
                    fire_flag = 1;
                    }
                break;

            case 103: //进行第一次绕杆
            if(flag == 0){
                //flag = move1.pole_rotation(rounding_pose[0],vel_pub,rate);
                flag = move1.fly_to_target(target_position,vel_pub,rate);
             }else{
                    last_time = ros::Time::now();
                    // double theta = atan((current_pose.pose.position.y - rounding_pose[0].pose.position.y)/(current_pose.pose.position.x - rounding_pose[0].pose.position.x));
                    // if(theta<0){
                    //     theta+=M_PI;
                    // }
                    // ROS_INFO("angle = %.2lf",theta);
                //    move1.circle_init(sqrt((rounding_pose[0].pose.position.x - rounding_pose[2].pose.position.x)*(rounding_pose[0].pose.position.x - rounding_pose[2].pose.position.x)+
                //     (rounding_pose[0].pose.position.y - rounding_pose[2].pose.position.y)*(rounding_pose[0].pose.position.y - rounding_pose[2].pose.position.y))
                //     ,0.25,theta,2.5*M_PI);//设定第一次绕杆的位置
                    move1.circle_init(0.3,0.25,M_PI,2*M_PI);
                    ROS_INFO("step104");
                    step = 104;
                    flag = 0;
                }
            break;
            case 104:
            if(flag == 0){
                flag = move1.pole_rotation(rounding_pose[0],vel_pub,rate);
             }else{
                 move1.pid_init();
                    move1.set_pid_all(0.8,0,0);
                    move1.set_pid_z(1.2,0.2,0.1);
                    move1.set_height(TargeyHeight);
                    last_time = ros::Time::now(); //等待接收杆的数据
                    rounding_pose[0] = current_pose;
                    target_position = move1.get_interupt_pose();
                    step = move1.get_interupt_step();
                    step --;
                    ROS_INFO("back to step %d",step);
                    flag = 0;
                    fire_flag = 1;
                }
            break;

            }
            }
        }
        rate.sleep();
    }
    return 0;
}