#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private)                                        
                                                                                                                      {
    i2cSetup();
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
