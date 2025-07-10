//
// Created by Administrator on 25-7-3.
//

#ifndef PROGRAM_HPP
#define PROGRAM_HPP


#include "Config/Config.hpp"
#include "Peripheral/GPIO.hpp"
#include "Peripheral/uart.hpp"
#include "Peripheral/TIM.hpp"

#include <cstdint>

extern const Peripheral::Uart<Peripheral::Interrupt> bus;
extern const Peripheral::Uart<Peripheral::Interrupt> dataPort;
extern const Peripheral::Uart<Peripheral::Normal> vision;
extern const Peripheral::Uart<Peripheral::Normal> debugPort;
extern const Peripheral::Uart<Peripheral::Normal> bakPort;
extern const Peripheral::PwmChannel<Peripheral::Normal> pitch;
extern const Peripheral::PwmChannel<Peripheral::Normal> yaw;
extern const Peripheral::PwmChannel<Peripheral::Normal> pump;
extern const Peripheral::Timer<Peripheral::Interrupt> timer;

#endif //PROGRAM_HPP
