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

extern const Peripheral::Uart<Peripheral::DMA> bus;
extern const Peripheral::Uart<Peripheral::Interrupt> dataPort;
extern const Peripheral::Uart<Peripheral::Normal> vision;
extern const Peripheral::Uart<Peripheral::Normal> debugPort;
extern const Peripheral::Uart<Peripheral::Normal> bakPort;
extern const Peripheral::PwmChannel<Peripheral::Normal> pitch;
extern const Peripheral::PwmChannel<Peripheral::Normal> yaw;
extern const Peripheral::PwmChannel<Peripheral::Normal> pump;
extern const Peripheral::Timer<Peripheral::Interrupt> timer;

enum {

    yawStandard   = 600,
    pitchLower    = 500,
    pitchStandard = 500,

    pitchLowest = 550, //40% 
    plS = 230,
    plH = 403,


    pitchHighest = 800, // 70%

    packetSize = 22,

    imageWidth = 768,
    imageHeight = 576

};


struct Point {
    int x;
    int y;
};


#endif //PROGRAM_HPP
