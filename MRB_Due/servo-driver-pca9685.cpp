#include "servo-driver-pca9685.hpp"

// Send a Restart signal to the PCA9685
void pca9685::reset(){
	uint8_t old_mode1 = read_reg(MODE1);
	
	if (old_mode1 & 0x10) { // Disable the SLEEP bit if it is 1, and wait for the oscillator to stabilize
		write_reg(MODE1, old_mode1 & 0xEF);
		hwlib::wait_us(500);
	}
	
	write_reg(ALL_LED_ON_L + 3, 0x10); // Set all duty cycles to 0%
	write_reg(MODE1, 0xA1); // Set the MODE1 register to restart, enable Auto-Increment and support ALLCALL.
}

// Send a Software Restart signal to ALL PCA9685 devices on the I2C bus.
void pca9685::reset_all_devices(){
	uint8_t reset_data[] = {SW_RESET};
	bus.write(0x00, reset_data, 1);
	
	hwlib::wait_us(10);
}

void pca9685::setPWMfreq(float freq){
	freq *= 0.885; // Corrector so the actual frequency will be closer to the one set.
	uint16_t pre_scale_val = (25000000 / (4096 * freq)) - 1;
	if (pre_scale_val > 255) pre_scale_val = 255;
    if (pre_scale_val < 3) pre_scale_val = 3;
	
	uint8_t mode1_old = read_reg(MODE1); // Read the MODE1 register
	
	write_reg(MODE1, mode1_old | 0x10); // Enables the SLEEP bit in the MODE1 register
	write_reg(PRESCALE, pre_scale_val); // Write the new frequency to the PRESCALE register
	write_reg(MODE1, (mode1_old | 0x20) & 0x6F); // Re-enable Auto-increment and set the SLEEP bit to 0
	
	hwlib::wait_us(500); // Wait for the oscillator to restart properly after setting the SLEEP bit to 0.
}

void pca9685::setPWM(const uint8_t servo_num, const uint16_t start_time, const uint16_t stop_time){
	uint8_t data[] = {uint8_t(LED0_ON_L + 4 * servo_num), uint8_t(start_time), uint8_t(start_time>>8), uint8_t(stop_time), uint8_t(stop_time>>8)};
	bus.write(address, data, sizeof(data) / sizeof(uint8_t));
}


// Read one register from PCA9685
uint8_t pca9685::read_reg(uint8_t reg_addr){
	uint8_t data[] = {reg_addr};
	bus.write(address, data, sizeof(data) / sizeof(uint8_t));
	bus.read(address, data, 1);
	return data[0];
}


// Write one register from PCA9685
void pca9685::write_reg(uint8_t reg_addr, uint8_t data){
	uint8_t send_data[] = {reg_addr, data};
	bus.write(address, send_data, sizeof(send_data) / sizeof(uint8_t));
}