//
// Created by Administrator on 25-7-3.
//

#include "Program.hpp"
#include "Config/Config.hpp"
#include "Task.hpp"

#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"

#include "tim.h"
#include "usart.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace Peripheral;
using mv = Platform::Chassis::MoveDirection;
using namespace Platform::Chassis;

const Uart<Interrupt> bus(&huart5);
const Uart<Interrupt> dataPort(&huart1);
const Uart<Normal> vision(&huart3);
const Uart<Normal> debugPort(&huart6);
const Uart<Normal> bakPort(&huart2);
const PwmChannel<Normal> pitch(&htim3, TIM_CHANNEL_1);
const PwmChannel<Normal> yaw(&htim3, TIM_CHANNEL_3);
const PwmChannel<Normal> pump(&htim5, TIM_CHANNEL_1);
const Timer<Interrupt> timer(&htim1);

enum {
    yawStandard   = 600,
    pitchLower    = 400,
    pitchStandard = 500
};


extern "C" [[noreturn]] void Init()
{
    static GPIOPin<Output> flowControlPin(GPIOB, GPIO_PIN_3);
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
    MecanumChassis<Platform::Chassis::RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, 37.5, 135.3);
#if 0
    HAL_Delay(3000);

    MCRSBPtr->RunTask(mv::Rotate,180,500);

    HAL_Delay(5000);

    //Platform::Chassis::MCRSBPtr->RunTask(mv::Forward, 10, 500);
#endif
    for (;;) {
        while (Task::GetIsMoving()) {
            Task::ReceiveBuffer();
        }
        for (uint32_t i = 0; i < Task::taskSize; i++) {
            if (Task::tasks[i].type == Task::TaskType::Inject) {
                auto duration = Task::tasks[i].content.injectDuration;
                Task::JetWater(duration);
            } 
            else {
                auto [direction,distance] = Task::tasks[i].content.move;
                MCRSBPtr->RunTaskTime(direction, distance);
            }
        }
        Task::TaskClear();
        // dataPort.ReceiveIdleIT(buffer, 1024);
    }
}
