#include "Connect.hpp"
#include "Platform/Chassis/Mecanum/RS485Bus/RS485Bus.hpp"
#include "Program.hpp"

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == bus.handle->Instance) {
        Platform::Chassis::MCRSBPtr->VerifyReached();
    }
}

