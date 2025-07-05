//
// Created by Administrator on 25-7-3.
//

#ifndef PROGRAM_HPP
#define PROGRAM_HPP


#include "Config/Config.hpp"
#include "Peripheral/GPIO.hpp"
#include "Peripheral/uart.hpp"
#include "usart.h"
#include <cstdint>

extern const Peripheral::Uart<Peripheral::Interrupt> bus;
extern const Peripheral::Uart<Peripheral::Interrupt> dataPort;
extern const Peripheral::Uart<Peripheral::Normal> vision;
extern const Peripheral::Uart<Peripheral::Normal> debugPort;
extern const Peripheral::Uart<Peripheral::Normal> bakPort;

void TaskCallBack(uint32_t Size);

#endif //PROGRAM_HPP
