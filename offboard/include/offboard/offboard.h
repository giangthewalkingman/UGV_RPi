#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<iostream>
#include<cmath>
#include<cstdio>
#include<vector>
#include<wiringPi.h>
#include<wiringPiI2C.h>

#define DEVICE_ID 0x08

class OffboardControl
{
    public:
    OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint);
    ~OffboardControl();
    private:
    ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

    void offboard();
    void i2cSetup();
    void waitForArming();
};

#endif