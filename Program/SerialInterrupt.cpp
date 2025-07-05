#include "Connect.hpp"
#include "Program.hpp"

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == dataPort.handle->Instance) {
        dataPortCallBack(Size);
    }
}

