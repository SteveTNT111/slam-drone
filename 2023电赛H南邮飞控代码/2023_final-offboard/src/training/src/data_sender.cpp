#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;
 
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64MultiArray>("/lidar/gan", 100);
 
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        msg.data.push_back(1);//自己写的，可行
        msg.data.push_back(1.3432432);
        msg.data.push_back(0.3913435);
        msg.data.push_back(1.5839453);
        msg.data.push_back(2.0845343);
        msg.data.push_back(1.4578843);
        msg.data.push_back(1.4898954);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}