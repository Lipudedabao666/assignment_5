//
// Created by Legion on 2026/11/8.
//

#include "LED_bsp.h"

#include "cmsis_os2.h"

void LED_bsp::LED_Bsp() {
    HAL_GPIO_WritePin(GPIOH, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);
    if (cnt%3==0) {
        HAL_GPIO_WritePin(GPIOH, LED_R_Pin, GPIO_PIN_SET);
    }
    else if (cnt%3==1) {
        HAL_GPIO_WritePin(GPIOH, LED_G_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(GPIOH, LED_B_Pin, GPIO_PIN_SET);
    }
    cnt++;
}

void led_task(void *argument)
{
    LED_bsp led;
    for (;;) {
        led.LED_Bsp();
        osDelay(500);
    }
}

