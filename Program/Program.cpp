//
// Created by Administrator on 25-7-3.
//

#include "Program.hpp"

#include "Config/Config.hpp"

#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "stm32f4xx_hal.h"
#include "sys/_intsup.h"
#include "tim.h"

#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"
#include <cstdint>

#include "Connect.hpp"

#include <cstdio>
#include <cstring>

using namespace Peripheral;

const Uart<Interrupt> bus(&huart5);
const Uart<Interrupt> dataPort(&huart1);
const Uart<Normal> vision(&huart3);
const Uart<Normal> debugPort(&huart6);
const Uart<Normal> bakPort(&huart2);

enum {
    yawStandard   = 600,
    pitchLower    = 400,
    pitchStandard = 500
};

enum MoveType {
    Move      = 0,
    Rotate    = 1,
    NegRotate = 2,
    Inject    = 3,
};

struct MoveTask {
    Platform::Chassis::MoveDirection direction;
    double distance;
};

Timer<Interrupt> timer(&htim1);

MoveTask tasks[32]    = {};
unsigned int taskSize = 0;
uint8_t buffer[1024]{};

bool isMoving = false;

PwmChannel<Normal> pitch(&htim3, TIM_CHANNEL_1);
PwmChannel<Normal> yaw(&htim3, TIM_CHANNEL_3);
PwmChannel<Normal> pump(&htim5, TIM_CHANNEL_1);

void TaskClear(void)
{
    taskSize = 0;
    isMoving = false;   
    for (auto &i : tasks) {
        i.direction = Platform::Chassis::MoveDirection::Forward;
        i.distance  = 0.0;
    }
}

void InjectWater(int _duration)
{
    pump.SetDutyCycle(0.5);
    Delay(_duration);
    pump.SetDutyCycle(0.0);
}

void TaskCallBack(uint32_t Size)
{
    if (isMoving) {
        return;
    }
    if (Size > 5) {
        for (uint32_t i = 0; i < Size; i++) {
            if (buffer[i] == 0) {
                continue; // 跳过填充字节
            }
            if (buffer[i] == 0x54 && buffer[i + 4] == 0x45) {
                tasks[taskSize].distance = static_cast<double>(buffer[i + 2]) * 256 + static_cast<double>(buffer[i + 3]);
                if (buffer[i + 1] == Move) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Forward;
                } else if (buffer[i + 1] == Rotate) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Rotate;
                } else if (buffer[i + 1] == NegRotate) {
                    tasks[taskSize].direction = Platform::Chassis::MoveDirection::Rotate;
                    tasks[taskSize].distance  = -tasks[taskSize].distance; // 反向旋转
                } else if (buffer[i + 1] == Inject) {
                    InjectWater(buffer[i+2] * 256 + buffer[i + 3]);
                    continue;
                }
                taskSize++;
                i += 4;
            }
        }
        if (taskSize > 0) {
            isMoving = true;
            char bufferx[64];
            sprintf(bufferx, "Received %u tasks\n", taskSize);
            dataPort.Send(bufferx, strlen(bufferx));
        } else {
            dataPort.Send("Error: No valid task received.\n", 30);
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

    constexpr uint8_t address[] = {1, 2, 3, 4};
    constexpr bool dirs[]       = {true, true, true, false};

    // constexpr uint8_t address[] = {3, 4, 1, 2};  // 左后、右后、左前、右前
    // constexpr bool dirs[] = {false, false, true, true};  // 反转所有电机的正方向

    pitch.Start();
    yaw.Start();

    pump.SetCompare(0);
    pump.Start();

    pitch.SetCompare(pitchStandard);
    yaw.SetCompare(yawStandard);

    // WaitForConnect();
    timer.StartIT();
    Platform::Chassis::MecanumChassis<Platform::Chassis::RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, 37.5, 135.3);
#if 0
    HAL_Delay(3000);

    Platform::Chassis::MCRSBPtr->RunTask(mv::Rotate,180,500);

    HAL_Delay(5000);

    //Platform::Chassis::MCRSBPtr->RunTask(mv::Forward, 10, 500);
#endif
    for (;;) {
        while (!isMoving) {
            dataPort.Receive(buffer, 1024, 1000);
            TaskCallBack(1024);
        }
        for (uint32_t i = 0; i < taskSize; i++) {
            Platform::Chassis::MCRSBPtr->RunTaskTime(tasks[i].direction, tasks[i].distance);
        }
        TaskClear();
        // dataPort.ReceiveIdleIT(buffer, 1024);
    }
}
