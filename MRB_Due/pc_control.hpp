#ifndef MRB_PC_CONTROL_HPP
#define MRB_PC_CONTROL_HPP

#include "hwlib.hpp"
#include "servo-driver-pca9685.hpp"


class pc_control{
private:
    pca9685 & servo_driver;
    const char * cmd_prefix = "Cmd";

    void setDutyCycle(uint8_t servo_num, uint16_t stop_time);

public:
    pc_control(pca9685 & servo_driver):
        servo_driver(servo_driver)
    {}

    void process();
};


#endif
