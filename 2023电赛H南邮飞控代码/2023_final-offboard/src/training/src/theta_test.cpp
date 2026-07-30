// 7/31 11:13
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
#define TargeyHeight 1.43
#define TargetYaw 0.2
#define TargetRadius 0.5
#define Limitv_x 0.5
#define Limitv_y 0.5
#define Limitv_z 0.5
#define Limit_yaw 0.5

ros::Subscriber state_sub;//飞机当前状态cur
ros::Subscriber pose_sub;//飞机当前位置信息
ros::Subscriber vel_sub;//飞机速度信息
ros::Subscriber pole_pose_sub;
ros::Publisher local_pose_pub;//设置飞机目标位置（全局坐标
ros::Publisher vel_pub;//设置飞机速度（全局坐标
ros::Publisher state_pub;//飞机状态
ros::Publisher set_raw_pub; //以飞机自己为坐标系的速度控制
ros::Publisher hit_pub;
ros::Publisher laser_pub;
ros::Publisher state_message_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
geometry_msgs::TwistStamped vel_cmd; //全局消息，速度设置
geometry_msgs::PoseStamped pose;//全局消息，分别用来控制飞机自身的位置与飞机降落add_executable(pole_test src/pole_test.cpp)
int land_distance = -1;
double last_error_yaw=0, last_error_dist=0;
double yaw_i = 0 , dist_i = 0;
bool cur_state;

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


// std_msgs::ColorRGBA pole_pose;
// void pose_pose_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
// {
//     pole_pose = *msg;
// }

std_msgs::Float64MultiArray pole_circle_pose;
void pose_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    pole_circle_pose = *msg;
}


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
        double vz = kp * ez + ki * iz + kd * dz;
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

    void velocity_land(ros::Rate& rate){  //降落程序中，pid_cal的序号为0
    mavros_msgs::PositionTarget raw_target;
	raw_target.coordinate_frame = 1;
    raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
    mavros_msgs::PositionTarget::IGNORE_PY | 
    mavros_msgs::PositionTarget::IGNORE_PZ |
    mavros_msgs::PositionTarget::IGNORE_YAW;
    raw_target.velocity.x = -current_pose.pose.position.x;
    raw_target.velocity.y = -current_pose.pose.position.y;
    raw_target.velocity.z = -current_pose.pose.position.z - 0.2;
    //ROS_INFO("%.2lf",raw_target.velocity.z);
	set_raw_pub.publish(raw_target);
    }

    void set_pid_all(double p,double i,double d){
        kp = p;
        ki = i;
        kd = d;
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
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>
        ("/iris_0/mavros/state", 10, state_cb);  //订阅状态
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/iris_0/mavros/local_position/pose", 10, pose_cb); //订阅位置信息.
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped> 
        ("/iris_0/mavros/local_position/velocity", 10 ,vel_cb);
    //pole_pose_sub = nh.subscribe<std_msgs::ColorRGBA>//发送雷达节点数据，用来规定发送的角度
    //    ("/lidar/gan",10,pose_pose_cb);
    pole_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>//发送雷达节点数据，用来规定发送的角度
        ("/lidar/gan",100,pose_pose_cb);   

    
    // state_sub = nh.subscribe<mavros_msgs::State>
    //     ("/mavros/state", 10, state_cb);  //订阅状态
    // pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("/mavros/local_position/pose", 10, pose_cb); //订阅位置信息.
    // vel_sub = nh.subscribe<geometry_msgs::TwistStamped> 
    //     ("/mavros/local_position/velocity", 10 ,vel_cb);
    // pole_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>//发送雷达节点数据，用来规定发送的角度
    //     ("/lidar/gan",100,pose_pose_cb);   
    // 指定发布位置设定点的频率
    ros::Rate rate(50.0);  
    geometry_msgs::PoseStamped rounding_pose[3];
    move_map move1;
    while(ros::ok()){
        ros::spinOnce();
        if( pole_circle_pose.data.size()>0 && pole_circle_pose.data.at(0) > 0 ){
        rounding_pose[0].pose.position.x = pole_circle_pose.data.at(1);
        rounding_pose[0].pose.position.y = pole_circle_pose.data.at(2);
        rounding_pose[1].pose.position.x = pole_circle_pose.data.at(3);
        rounding_pose[1].pose.position.y = pole_circle_pose.data.at(4);
        rounding_pose[2].pose.position.x = pole_circle_pose.data.at(5);
        rounding_pose[2].pose.position.y = pole_circle_pose.data.at(6);
         double theta = atan((current_pose.pose.position.y - rounding_pose[1].pose.position.y)/(current_pose.pose.position.x - rounding_pose[1].pose.position.x));
        if(current_pose.pose.position.x < rounding_pose[1].pose.position.x && theta < 1.57){
            theta+=3.14159;
        }
        if(theta < 0 && current_pose.pose.position.x < rounding_pose[1].pose.position.x){
            theta+=3.14159;
        }
        if(theta < 0 && current_pose.pose.position.x > rounding_pose[1].pose.position.x){
            theta+=3.14159*2;
        }
        ROS_INFO("theta = %.2lf",theta);
        }
        rate.sleep();
    }
    return 0;
}