//
// Created by Administrator on 25-7-3.
//

#include "Program.hpp"
#include "Task.hpp"
#include "Action.hpp"

#include "Config/Config.hpp"
#include "Peripheral/Mode.hpp"
#include "Peripheral/TIM.hpp"
#include "Platform/Chassis/ChassisU.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"

#include "tim.h"
#include "usart.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdint.h>

using namespace Peripheral;
using namespace Platform::Chassis;

const Uart<DMA> bus(&huart5);
const Uart<Interrupt> dataPort(&huart1);
const Uart<Normal> vision(&huart3);
const Uart<Normal> debugPort(&huart6);
const Uart<Normal> bakPort(&huart2);
const PwmChannel<Normal> pitch(&htim3, TIM_CHANNEL_1);
const PwmChannel<Normal> yaw(&htim3, TIM_CHANNEL_3);
const PwmChannel<Normal> pump(&htim5, TIM_CHANNEL_1);
const Timer<Interrupt> timer(&htim1);

void Init()
{
    static GPIOPin<Output> flowControlPin(GPIOB, GPIO_PIN_3);
    flowControlPin.Write(false);

    HAL_Delay(2000);
    // flowControlPin.Write(true);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

    constexpr uint8_t address[] = {1, 2, 3, 4};
    constexpr bool dirs[]       = {true, true, true, false};

    pitch.Start();
    yaw.Start();

    pump.SetCompare(0);

    ServoReset();

    // WaitForConnect();
    timer.StartIT();
    MecanumChassis<Platform::Chassis::RS485Bus>::Create(
        address, &bus, flowControlPin, 16, dirs, true, 37.5, 135.3);
}

extern "C" [[noreturn]] void Main()
{

    Init();


#if 0
    HAL_Delay(3000);

    MCRSBPtr->RunTask(mv::Rotate,180,500);

    HAL_Delay(5000);

    //Platform::Chassis::MCRSBPtr->RunTask(mv::Forward, 10, 500);
#endif
    

    for (;;) {
        while (!Task::GetIsMoving()) {
            Task::ReceiveBuffer();
        }
        for (uint32_t i = 0; i < Task::taskSize; i++) {
            auto [direction, distance] = Task::tasks[i].content.move;
            MCRSBPtr->RunTaskTime(direction, distance);
            dataPort.Send("S", 1);
        }
        Task::TaskClear();
        AimFire();
        // dataPort.ReceiveIdleIT(buffer, 1024);
    }
}
