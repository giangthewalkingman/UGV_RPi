#include "offboard/offboard.h"

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private)                                        
                                                                                                                      {
    arm_mode_sub = nh_.subscribe("arm_mode", 10, &OffboardControl::armModeCallback, this);
    nh_private_.param<bool>("/offboard_node/arm_mode_enable", arm_mode_.data);
    
    operation_time_1 = ros::Time::now();
    i2cSetup();
    waitForArming(10);
    offboard();
}

//destructor
OffboardControl::~OffboardControl() {
}

void OffboardControl::offboard() {
    std::printf("[ INFO] Armed. \n");
    std::printf("Choose mode: \n");
    std::printf("(1) Control with keyboard: \n");
    std::printf("(2) Mission: \n");
    std::printf("(3) Cancel: \n");
    int mode;
    std::cin >> mode;
    switch (mode)
    {
    case 1:
        teleopControl();
        break;
    
    default:
        break;
    }
}

void OffboardControl::i2cSetup() {
    fd = wiringPiI2CSetup(DEVICE_ID);
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
}

void OffboardControl::teleopControl() {
    initNcurses(); // Initialize ncurses

    int ch;
    while ((ch = getch()) != 'q') { // Loop until 'q' is pressed
        switch(ch) {
            case KEY_UP:
                sendI2CMsg(pwmValue[0], pwmValue[1], pwmValue[2], pwmValue[3], FORWARD);
                std::cout << "Forward\n";
                // Adjust PWM values and direction for forward motion
                break;
            case KEY_DOWN:
                sendI2CMsg(pwmValue[0], pwmValue[1], pwmValue[2], pwmValue[3], BACKWARD);
                std::cout << "Backward\n";
                // Adjust PWM values and direction for backward motion
                break;
            case KEY_LEFT:
                sendI2CMsg(pwmValue[0], pwmValue[1], pwmValue[2], pwmValue[3], TURNLEFT);
                std::cout << "Left\n";
                // Adjust PWM values and direction for left turn
                break;
            case KEY_RIGHT:
                sendI2CMsg(pwmValue[0], pwmValue[1], pwmValue[2], pwmValue[3], TURNRIGHT);
                std::cout << "Right\n";
                // Adjust PWM values and direction for right turn
                break;
            default:
                std::cout << "Invalid input\n";
                break;
        }
    }

    cleanupNcurses(); // Cleanup ncurses
    landing();
}

void OffboardControl::armModeCallback(const std_msgs::Bool::ConstPtr &msg) {
    arm_mode_ = *msg;
}

// Function to initialize ncurses and keyboard input
void OffboardControl::initNcurses() {
    initscr(); // Initialize ncurses
    cbreak();  // Line buffering disabled
    noecho();  // Don't echo any keypresses
    keypad(stdscr, TRUE); // Enable keypad mode for arrow keys
}

// Function to cleanup ncurses
void OffboardControl::cleanupNcurses() {
    endwin(); // End ncurses
}

void OffboardControl::landing() {
    sendI2CMsg(0, 0, 0, 0, STOP);
    operation_time_2 = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2 - operation_time_1).toSec());
    ros::shutdown();
}

void OffboardControl::sendI2CMsg(uint8_t right_front_pwm, uint8_t left_front_pwm, uint8_t right_back_pwm, uint8_t left_back_pwm, uint8_t direction) {
    wiringPiI2CWrite(fd, right_front_pwm);
    wiringPiI2CWrite(fd, left_front_pwm);
    wiringPiI2CWrite(fd, right_back_pwm);
    wiringPiI2CWrite(fd, left_back_pwm);
    wiringPiI2CWrite(fd, direction);
}