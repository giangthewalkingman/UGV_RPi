#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<ros/time.h>
#include<iostream>
#include<cmath>
#include<cstdio>
#include<vector>
#include<unistd.h>
#include<ncurses.h>
#include<curses.h>
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

    int16_t pwmArray[4];
    ros::Time operation_time_1, operation_time_2;

    void armModeCallback(const std_msgs::Bool::ConstPtr &msg);

    void offboard();
    void landing();
    void teleopControl();
    void i2cSetup();
    void waitForArming(double hz);
    void initNcurses();
    void cleanupNcurses();

};

#endif