#include "hwlib.hpp"
#include "servo-driver-pca9685.hpp"
#include "pc_control.hpp"


int main(){
	// kill the watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	hwlib::wait_ms(200);
	
	auto scl = hwlib::target::pin_oc(hwlib::target::pins::scl);
	auto sda = hwlib::target::pin_oc(hwlib::target::pins::sda);
	auto bus = hwlib::i2c_bus_bit_banged_scl_sda(scl, sda);
	auto servo_driver = pca9685(bus, 0x40);

    auto controller = pc_control(servo_driver);

	controller.process();
}
