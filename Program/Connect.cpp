
#include "Program.hpp"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <cstdint>



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

void dataPortCallBack(uint32_t Size)
{
    
    if (!connected) {  
        for (auto i : connectBuffer) {
            if (i == 'R') {
                connected = true;
                return;
            }
        }
    }
    else {
        TaskCallBack(Size);
    }
    dataPort.ReceiveIdleIT((uint8_t *)connectBuffer, 32);
}