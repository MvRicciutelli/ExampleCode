/**
 * @file MCAL_Gpio.hpp
 * @brief header for the implementation of a low level abstraction of gpio
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 
 */

#ifndef STM32H5XX_MCAL_GPIO_HPP_
#define STM32H5XX_MCAL_GPIO_HPP_
#include <MCAL_iGpio.hpp>

#if defined( STM32H563xx ) || defined( STM32H573xx)
#   include "stm32h5xx_hal.h"
#endif

namespace MCAL {
typedef struct
{
	std::uint32_t pin;
	GPIO_TypeDef* port;
}GpioInit_t;

class Gpio:public iGpio {
public:
				 Gpio(const GpioInit_t& _init);
	void 		 Write(std::uint8_t val) override;
	std::uint8_t Read() 				 override;
private:
	GpioInit_t init;
};

} /* namespace APP */

#endif /* STM32H5XX_MCAL_GPIO_HPP_ */
