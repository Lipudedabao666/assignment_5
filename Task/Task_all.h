//
// Created by Legion on 2026/11/8.
//

#ifndef DX_ASSIGNMENT5_FREERTOS_TASK_ALL_H
#define DX_ASSIGNMENT5_FREERTOS_TASK_ALL_H

#include "main.h"

class Task_all {
public:

private:

};

typedef struct
{
    float    channel[4];
    uint8_t  s1;
    uint8_t  s2;
    uint32_t last_frame_tick;
    uint8_t  received_once;
} DRV_RemoteControl_State_t;

#ifdef __cplusplus
extern "C" {
#endif

void imu_task(void *argument);
void dji_motor_driver(void *argument);
void power_receiver(void *argument);
void remote_controler(void *argument);

#ifdef __cplusplus
}
#endif

#endif //DX_ASSIGNMENT5_FREERTOS_TASK_ALL_H
