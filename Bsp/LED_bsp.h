//
// Created by Legion on 2026/11/8.
//

#ifndef DX_ASSIGNMENT5_FREERTOS_LED_BSP_H
#define DX_ASSIGNMENT5_FREERTOS_LED_BSP_H
#include <cstdint>

#include "main.h"

class LED_bsp {
public:
    LED_bsp() : cnt(0) {}
    void LED_Bsp();
private:
    uint32_t cnt;
};

#ifdef __cplusplus
extern "C" {
#endif

void led_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif //DX_ASSIGNMENT5_FREERTOS_LED_BSP_H
