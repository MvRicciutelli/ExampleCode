/*
 * MCAL_iUart.hpp
 *
 *  Created on: 22/12/2023
 */
#ifndef STM32H5XX_MCAL_iUart_H_
#define STM32H5XX_MCAL_iUart_H_
#include <cstdint>

using namespace std;
namespace MCAL
{

class iUart
{
public:
	virtual bool TransmitIrqStart(uint8_t* txData, uint32_t txLength) = 0;
	virtual bool TransmitBlocking(uint8_t *txData, uint32_t txLength, uint32_t timeout_ms) = 0;
	virtual bool TransmitIrqStop() = 0;
	virtual bool ReceiveIrqStart(uint8_t* rxData, uint32_t rxLength) = 0;
	virtual bool ReceiveIrqStop() = 0;
};

}

#endif
