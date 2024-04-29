#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private)                                        
                                                                                                                      {
    arm_mode_sub = nh_.subscribe("arm_mode", 10, &OffboardControl::armModeCallback, this);
    nh_private_.param<bool>("/offboard_node/arm_mode_enable", arm_mode_.data);
    i2cSetup();
    waitForArming(10);
    offboard();
}

//destructor
OffboardControl::~OffboardControl() {
}

void OffboardControl::offboard() {

}

void OffboardControl::i2cSetup() {
    int fd = wiringPiI2CSetup(DEVICE_ID);
    if (fd == -1) {
        std::cout << "Failed to init I2C communication.\n";
    } else {
        std::cout << "I2C communication successfully setup.\n";
    }
}

void OffboardControl::waitForArming(double hz) {
    ros::Rate rate(hz);
    std::printf("[ INFO] Waiting for Arming... \n");
    while(ros::ok() && arm_mode_.data == false) {
        rate.sleep();
        ros::spinOnce();
    }
    std::printf("[ INFO] Armed. \n");
}

void OffboardControl::armModeCallback(const std_msgs::Bool::ConstPtr &msg) {
    arm_mode_ = *msg;
}