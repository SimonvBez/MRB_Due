#include "pc_control.hpp"

void pc_control::setDutyCycle(const uint8_t servo_num, const uint16_t stop_time) {
    if (stop_time < 4096 && servo_num <= 3){
        servo_driver.setPWM(servo_num, 0, stop_time);
    }
}


void pc_control::process() {
    char serial_input;

    while (true){
        uint8_t input_counter = 0;
        while (true){
            // Wait for the command prefix to be received.
            hwlib::cin >> serial_input;

            if (serial_input == cmd_prefix[input_counter]) {
                input_counter++;

                if (input_counter == 3) {   // 3 is the length of the command prefix.
                    break;
                }
            } else {
                input_counter = 0;
            }
        }

        hwlib::cin >> serial_input;
        switch(serial_input){
            case 'R': {
                // Commando 'R': Resets the PCA9685 chip
                servo_driver.reset();
                servo_driver.setPWMfreq(62.5);
                break;
            }

            case 'S':{
                // Command 'S': Sets the specified servo to a given stop time.
                uint8_t servo_num;
                uint16_t stop_time;

                hwlib::cin >> serial_input;
                servo_num = reinterpret_cast<uint8_t &>(serial_input); // Get the servo number to be set.

                hwlib::cin >> serial_input;
                stop_time = reinterpret_cast<uint8_t &>(serial_input) << 8; // Get the first/left byte of the stop_time
                hwlib::cin >> serial_input;
                stop_time |= reinterpret_cast<uint8_t &>(serial_input); // Get the second/right byte of the stop_time

                servo_driver.setPWM(servo_num, 0, stop_time);
                break;
            }

            case 'A':{
                // Command 'A': Sets all servo's
                uint16_t stop_times[3];
                for (uint16_t & stop_time : stop_times){ // Receive bytes
                    hwlib::cin >> serial_input;
                    stop_time = reinterpret_cast<uint8_t &>(serial_input) << 8; // Get the first/left byte of the stop_time
                    hwlib::cin >> serial_input;
                    stop_time |= reinterpret_cast<uint8_t &>(serial_input); // Get the second/right byte of the stop_time
                }

                for (uint8_t i = 0; i < 3; i++){ // Apply received bytes
                    servo_driver.setPWM(i+1, 0, stop_times[i]);
                }
                break;
            }

            default:{
                break;
            }
        }
    }
}
