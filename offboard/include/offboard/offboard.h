#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<iostream>
#include<cmath>
#include<cstdio>
#include<vector>
#include<wiringPi.h>
#include<wiringPiI2C.h>
#include<std_msgs/Bool.h>

#define DEVICE_ID 0x08

class OffboardControl
{
    public:
    OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint);
    ~OffboardControl();
    private:
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

    ros::Subscriber arm_mode_sub;

    std_msgs::Bool arm_mode_;

    void armModeCallback(const std_msgs::Bool::ConstPtr &msg);

    void offboard();
    void i2cSetup();
    void waitForArming(double hz);
};

#endif