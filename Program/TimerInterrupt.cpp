#include "Peripheral/TIM.hpp"
#include "Peripheral/GPIO.hpp"


#include "tim.h"
#include "Config/Config.hpp"


using namespace Peripheral;

const GPIOPin<Output> led (GPIOA, GPIO_PIN_12);

extern Timer<Interrupt> timer;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == timer.handle->Instance) {
        led.Toggle();
    }
}