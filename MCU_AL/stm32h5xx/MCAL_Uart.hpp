/**
 * @file MCAL_Uart.hpp
 * @brief CPP implementation of a low level abstraction of uart
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 
 */
#ifndef STM32H5XX_MCAL_UART_HPP_
#define STM32H5XX_MCAL_UART_HPP_
#include <MCAL_iUart.hpp>

#if defined( STM32H563xx ) || defined( STM32H573xx)
#   include "stm32h5xx_hal.h"
#endif

namespace MCAL {

/**
 * @brief class abstraction for a uart peripheral, implemented for a stm32h5xx microcontroller
 *
 */
class Uart:public iUart {
public:
    Uart(UART_HandleTypeDef* _huart);
    bool TransmitIrqStart(uint8_t* txData, uint32_t txLength);
    bool TransmitBlocking(uint8_t* txData, uint32_t txLength, uint32_t timeout_ms);
    bool TransmitIrqStop();
    bool ReceiveIrqStart(uint8_t* rxData, uint32_t rxLength);
    bool ReceiveIrqStop();
private:
    ///@brief stm32 hal uart handle pointer
    UART_HandleTypeDef* huart;
};

} /* namespace APP */

#endif /* STM32H5XX_MCAL_GPIO_HPP_ */
