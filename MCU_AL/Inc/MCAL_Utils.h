/** @file MCAL_Utils.h
* @brief  Header file for Utilities
* @author Matteo Vittorio Ricciutelli 
* @date   22/12/2023
*/
#ifndef MCU_AL_INC_MCAL_UTILS_H_
#define MCU_AL_INC_MCAL_UTILS_H_

///@brief privately back up primask bit storing it for later use,
/// @attention the caller function must also close any end to the critical section using UTILS_EXIT_CRITICAL
#define UTILS_ENTER_CRITICAL()                                  \
                uint32_t primask_bit = __get_PRIMASK();         \
                __disable_irq()


#define UTILS_EXIT_CRITICAL()                                   \
                __set_PRIMASK(primask_bit)                      \



#define __ASM __asm /*!< asm keyword for GNU Compiler */
#define __INLINE inline /*!< inline keyword for GNU Compiler */
#define __STATIC_INLINE static inline




#endif /* MCU_AL_INC_MCAL_UTILS_H_ */
