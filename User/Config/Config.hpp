//
// Created by Administrator on 25-3-2.
//

#ifndef CONFIG_HPP
#define CONFIG_HPP

#ifdef STM32H7xx
#include "stm32h7xx_hal.h"
#endif

#ifdef STM32F1xx
#include "stm32f1xx_hal.h"
#endif

#ifdef STM32F405xx
#include "stm32f4xx_hal.h"
#endif

#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif


#if CMSIS_RTOS
#include "cmsis_os2.h"
#endif

inline void Delay(uint32_t _ticks) {
#if CMSIS_RTOS
    osDelay(_ticks);
#else
    HAL_Delay(_ticks);
#endif
}

#endif //CONFIG_HPP
