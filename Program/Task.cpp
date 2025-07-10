
#include "Task.hpp"
#include "Program.hpp"

#include <cstdio>
#include <cstring>
#include <span>

#include "usart.h"

#include "Platform/Chassis/ChassisU.hpp"


namespace Task
{
    Task tasks[32]        = {};
    unsigned int taskSize = 0;
    uint8_t buffer[1024]{};

    bool isMoving = false;

    void TaskCallBack()
    {
        if (isMoving) {
            return;
        }
        for (uint32_t i = 0; i < 1024; i++) {
            if (buffer[i] == 0) {
                continue; // 跳过填充字节
            }
            if (buffer[i] == 0x54 && buffer[i + 4] == 0x45) {
                tasks[taskSize].content.move.distance = static_cast<double>(buffer[i + 2]) * 256 + static_cast<double>(buffer[i + 3]);
                if (buffer[i + 1] == Move) {
                    tasks[taskSize].content.move.direction = Platform::Chassis::MoveDirection::Forward;
                } else if (buffer[i + 1] == Rotate) {
                    tasks[taskSize].content.move.direction = Platform::Chassis::MoveDirection::Rotate;
                } else if (buffer[i + 1] == NegRotate) {
                    tasks[taskSize].content.move.direction = Platform::Chassis::MoveDirection::Rotate;
                    tasks[taskSize].content.move.distance  = -tasks[taskSize].content.move.distance; // 反向旋转
                } else if (buffer[i + 1] == Inject) {
                    tasks[taskSize].type                   = TaskType::Inject;
                    tasks[taskSize].content.injectDuration = buffer[i + 2] * 256 + buffer[i + 3];
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
        for (auto &i : buffer) {
            i = 0;
        }
    }

    void JetWater(int _duration)
    {
        pump.SetDutyCycle(0.5);
        Delay(_duration);
        pump.SetDutyCycle(0.0);
    }

    void TaskClear(void)
    {
        taskSize = 0;
        isMoving = false;
        for (auto &i : tasks) {
            i.type                   = TaskType::Move;
            i.content.move.direction = Platform::Chassis::MoveDirection::Forward;
            i.content.move.distance  = 0.0;
        }
    }

    static bool connected         = true;
    static char connectBuffer[32] = {0};

    void WaitForConnect()
    {
        // dataPort.ReceiveIdleIT((uint8_t *)connectBuffer);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)connectBuffer, 32);
        while (!connected) {
            dataPort.Send("T", 1);
            Delay(500);
        }
    }

    bool GetIsMoving()
    {
        return isMoving;
    }

    void ReceiveBuffer() {
        dataPort.Receive(buffer, 1024, 1000);
        TaskCallBack();
    }

}