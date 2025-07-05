//
// Created by Administrator on 25-7-3.
//

#include "Program.hpp"

#include "Config/Config.hpp"

#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"
#include <cstdint>

#include "Connect.hpp"

using namespace Peripheral;

const Uart<Interrupt> bus(&huart5);
const Uart<Interrupt> dataPort(&huart1);
const Uart<Normal> vision(&huart3);
const Uart<Normal> debugPort(&huart6);
const Uart<Normal> bakPort(&huart2);

enum MoveType {
    Move      = 0,
    Rotate    = 1,
    NegRotate = 2,
};

struct MoveTask {
    Platform::Chassis::MoveDirection direction;
    double distance;
};

Timer<Interrupt> timer(&htim1);

MoveTask tasks[32] = {};
uint32_t taskSize  = 0;
uint8_t buffer[1024]{};

bool isMoveing = false;

void TaskClear(void) {
    taskSize  = 0;
    isMoveing = false;
    for (auto &i : tasks) {
        i.direction = Platform::Chassis::MoveDirection::Forward;
        i.distance  = 0.0;
    }
}

void TaskCallBack(uint32_t Size)
{
    if (isMoveing) {
        return;
    }
    if (Size > 5) {
        for (uint32_t i = 0; i < Size; i++) {
            if (buffer[i] == 0) {
                continue; // 跳过填充字节
            }
            if (buffer[i] == 0x54 && buffer[i + 4] == 0x45) {
                tasks[taskSize].distance = static_cast<double>(buffer[i + 2]) * 255 + static_cast<double>(buffer[i + 3]) ;
                if (buffer[i+1] == Move) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Forward;
                } else if (buffer[i+1] == Rotate) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Rotate;
                } else if (buffer[i+1] == NegRotate) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Rotate;
                    tasks[taskSize].distance  = -tasks[taskSize].distance; // 反向旋转
                }
                taskSize++;
                i+=4;
            }
        }
        if (taskSize > 0) {
            isMoveing = true;
        }
    }
    for (auto &i : buffer) {
        i = 0;
    }
}

using mv = Platform::Chassis::MoveDirection;

extern "C" [[noreturn]] void Init()
{
    static Peripheral::GPIOPin<Peripheral::Output> flowControlPin(GPIOB, GPIO_PIN_3);
    flowControlPin.Write(false);

    HAL_Delay(2000);
    // flowControlPin.Write(true);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

    //constexpr uint8_t address[] = {1, 2, 3, 4};
    constexpr uint8_t address[] = {4, 3, 2, 1};

    //constexpr bool dirs[] = {true, true, true, false};
    constexpr bool dirs[] = {false, false, true, true};

    
    PwmChannel<Normal> pitch(&htim3, TIM_CHANNEL_1);
    PwmChannel<Normal> yaw(&htim3, TIM_CHANNEL_3);

    pitch.Start();
    yaw.Start();

    for (uint32_t i = 500; i < 2500; i+=100) {
        pitch.SetCompare(i);
        yaw.SetCompare(i);
    }
    pitch.Stop();
    yaw.Stop();
    //WaitForConnect();
    timer.StartIT();
    Platform::Chassis::MecanumChassis<Platform::Chassis::RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, 37.5, 135.3);
#if 0
    HAL_Delay(3000);


    
     Platform::Chassis::MCRSBPtr->RunTask(mv::Forward,100,500);

     HAL_Delay(5000);

    Platform::Chassis::MCRSBPtr->RunTask(mv::Forward, 10, 500);
#endif
    for (;;) {
        dataPort.Receive(buffer, 1024,  1000);
        TaskCallBack(1024);
        while (!isMoveing);
        for (uint32_t i = 0; i < taskSize; i++) {
            const auto t = Platform::Chassis::MCRSBPtr->RunTask(tasks[i].direction, tasks[i].distance, uint32_t((float)tasks[i].distance * 0.5));
            char c = t ? 'S' : 'F';
            dataPort.Send(&c, 1, 1000);
        }
        TaskClear();
        //dataPort.ReceiveIdleIT(buffer, 1024);
    }
}
