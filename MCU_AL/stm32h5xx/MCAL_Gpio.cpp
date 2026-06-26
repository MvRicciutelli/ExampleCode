/**
 * @file MCAL_Gpio.cpp
 * @brief CPP implementation of a low level abstraction of gpio
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 
 */

#include <MCAL_Gpio.hpp>

namespace MCAL {
/**
 * @brief constructor, it incorporates the gpio pin data
 *
 * @param _init init data
 */
Gpio::Gpio(const GpioInit_t& _init): init(_init) {
	// TODO Auto-generated constructor stub

}
/**
 * @brief writes a pin digital output
 *
 * @param 0 low pin off digital low, any other pin on (digital high)
 */
void Gpio::Write(std::uint8_t val)
{
	HAL_GPIO_WritePin(init.port, init.pin, (GPIO_PinState)val);
}

/**
 * @brief read a input pin digital value
 *
 * @return 1 pin on (digital high), 0 pin off- (digital low)
 */
std::uint8_t Gpio::Read()
{
	return (GPIO_PIN_SET == HAL_GPIO_ReadPin(init.port, init.pin));
}

} /* namespace APP */
