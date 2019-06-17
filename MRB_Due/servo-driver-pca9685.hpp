/// @file

#ifndef SERVO_DRIVER_PCA9685_HPP
#define SERVO_DRIVER_PCA9685_HPP

#include "hwlib.hpp"


/// \brief
/// PCA9685 16 Channel Servo Driver
/// \details
/// This class implements an interface to a 16 Channel Servo Driver.
/// The interface is I²C.
/// The driver chip is PCA9685.
/// 
/// The Vcc pin is used to power the chip, and has a voltage range of 2.3 V to 5.5V.
/// The V+ pin is used to power the servos. If you are controlling a lot of servos,
/// use an external power supply to prevent damage to the Arduino.
/// 
/// The Vcc and V+ pin have a common GND.
/// 
/// The OE (=Output Enable) pin is active LOW and should be connected to ground.
/// If you set the OE pin to HIGH, all outputs will be programmed to the value defined
/// in the OUTNE[1:0] in the MODE 2 register. (The default of OUTNE[1:0] will be to disable all outputs)
class pca9685 {
private:
	hwlib::i2c_bus & bus;
	uint_fast8_t address;
	
	// PCA9685 registers
	static constexpr const uint8_t MODE1 =			0x00;
	static constexpr const uint8_t MODE2 =			0x01;
	static constexpr const uint8_t SUB_ADR1 =		0x02;
	static constexpr const uint8_t SUB_ADR2 =		0x03;
	static constexpr const uint8_t SUB_ADR3 =		0x04;
	static constexpr const uint8_t ALL_CALL_ADR =	0x05;
	static constexpr const uint8_t LED0_ON_L =		0x06;
	static constexpr const uint8_t ALL_LED_ON_L =	0xFA;
	static constexpr const uint8_t PRESCALE =		0xFE;
	
	// PCA9685 software reset address
	static constexpr const uint8_t SW_RESET =		0x06;
	
	
	uint8_t read_reg(uint8_t reg_addr);
	void write_reg(uint8_t reg_addr, uint8_t data);
	
public:
	/// \brief
	/// Create a servo driver
	/// \details
	/// This constructor creates a servo driver from the I²C bus and makes it ready for use.
	pca9685(hwlib::i2c_bus & bus, const uint_fast8_t address = 0x40 ):
		bus(bus),
		address(address)
	{
		reset(); // Resets the PCA9685 and makes it ready for use.
		setPWMfreq(62.5); // Sets the default PWM frequency to 62.5Hz. (This could be anywhere between 100 to 50Hz)
	}
	
	/// \brief
	/// Set the PWM frequency
	/// \details
	/// This function sets the PCA9685's PWM to the given frequency (in Hz).
	/// The PCA9685 outputs operated from 24Hz to 1526Hz.
	/// 
	/// Because the frequency is stored in only one register byte (also known as the PRESCALE),
	/// the real frequency can be a few Hz of the one that is set.
	void setPWMfreq(float freq);
	
	/// \brief
	/// Set the PWM duty cycle
	/// \details
	/// This function sets a servo pin to a given duty cycle.
	/// 'servo_num' is the pin number you want to set (0 to 15).
	/// 'start_time' is the step when the output pin is set to HIGH (0 to 4096).
	/// 'stop_time' is the step when the outpit pin is set to LOW again (0 to 4096).
	/// 
	/// Example: setPWM(3, 0, 2048) will give pin 3 a duty cycle of 50%.
	/// 
	/// 
	/// Remember the angle of a servo motor is controlled by the *time* a PWM pulse
	/// is HIGH, and not just the duty cycle percentage.
	void setPWM(const uint8_t servo_num, const uint16_t start_time, const uint16_t stop_time);
	
	/// \brief
	/// Resets the PCA9685
	/// \details
	/// This function will reset the PCA9685.
	/// This means all registers will turn to their defaults.
	/// 
	/// Default PWM frequency: ~200Hz
	/// Default duty cycle: 0%
	void reset();
	
	/// \brief
	/// Resets all PCA9685's
	/// \details
	/// This function will reset all PCA9685's on the I²C bus
	/// and set their registers to their defaults.
	void reset_all_devices();
};


#endif