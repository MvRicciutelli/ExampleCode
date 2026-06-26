/**
 * @file MCAL_Uart.cpp
 * @brief CPP implementation of a low level abstraction of uart
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 
 */
#include <MCAL_Utils.h>
#include <MCAL_Uart.hpp>

namespace MCAL {

/**
 * @brief constructor that memorizes the st handle for the uart
 *
 * @param _huart handle
 */
Uart::Uart(UART_HandleTypeDef *_huart):huart(_huart)
{

}

/**
 * @brief Irq based transmission kick off, the data transfer will trigger and interrupt upon completion
 *
 * @param txData
 * @param txLength
 * @return
 */
bool Uart::TransmitIrqStart(uint8_t *txData, uint32_t txLength)
{
    return (HAL_OK == HAL_UART_Transmit_IT(huart, txData, txLength));
}


/**
 * @brief abort irq based transfer
 *
 * @return
 */
bool Uart::TransmitIrqStop()
{
    return (HAL_OK == HAL_UART_AbortTransmit_IT(huart));
}

/**
 * @brief kick off irq uart based data reception, it will
 * trigger an interrupt upon completion (all desired bytes received)
 * @param rxData
 * @param rxLength
 * @return
 */
bool Uart::ReceiveIrqStart(uint8_t *rxData, uint32_t rxLength)
{
    return (HAL_OK == HAL_UART_Receive_IT(huart, rxData, rxLength));
}

/**
 * @brief aborts irq based reception
 *
 * @return
 */
bool Uart::ReceiveIrqStop()
{
    return (HAL_OK == HAL_UART_AbortReceive_IT(huart));
}

/**
 * @brief starts a polling and blocking uart transfer with a timeout parameter
 *
 * @param txData
 * @param txLength
 * @param timeout_ms
 * @return
 */
bool Uart::TransmitBlocking(uint8_t *txData, uint32_t txLength, uint32_t timeout_ms)
{
    HAL_StatusTypeDef res = HAL_ERROR;
    UTILS_ENTER_CRITICAL();
    res =  HAL_UART_Transmit(huart, txData, txLength, timeout_ms);
    UTILS_EXIT_CRITICAL();
    return res == HAL_OK;
}

} /* namespace APP */
