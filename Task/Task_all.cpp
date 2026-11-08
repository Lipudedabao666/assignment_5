//
// Created by Legion on 2026/11/8.
//

#include "Task_all.h"
#include "motor_driver.h"
#include "DJI_motor_driver.h"
#include "bsp_uart.h"
#include "math.h"
#include "ins_task.h"

DRV_RemoteControl_State_t remote_control;

void imu_task(void *argument) {
    for (;;) {
        uint32_t tick=osKernelGetTickCount();
        INS_Task();
        tick++;
        osDelayUntil(tick);
    }
}

void dji_motor_driver(void *argument) {
    DJI_motor motor;
    for (;;) {
        uint32_t tick=osKernelGetTickCount();
        float t=tick/1000.0f;
        motor.motor_set_rounds(sin(t));
        tick++;
        osDelayUntil(tick);
    }
}

void power_receiver(void *argument) {
    uint8_t power[8];
    can_device_receive message(&hcan1, 0x101, nullptr);
    for (;;) {
        uint32_t tick=osKernelGetTickCount();
        for (uint8_t i=0;i<8;i++) {
            power[i]=message.rx_data[i];
        }
        tick++;
        osDelayUntil(tick);
    }
}

void remote_controler(void *argument) {
    uint8_t data[18];
    for (;;) {
        uint32_t tick=osKernelGetTickCount();
        if (BSP_UART_GetFrame(data)!=0) {
            remote_control.channel[0] = ( data[0]        | (data[1] << 8) )                   & 0x07FF;
            remote_control.channel[1] = ( (data[1] >> 3) | (data[2] << 5) )                   & 0x07FF;
            remote_control.channel[2] = ( (data[2] >> 6) | (data[3] << 2) | (data[4] << 10) ) & 0x07FF;
            remote_control.channel[3] = ( (data[4] >> 1) | (data[5] << 7) )                   & 0x07FF;
            remote_control.s1 = (data[5] >> 4) & 0x03;
            remote_control.s2 = (data[5] >> 6) & 0x03;
        }
        tick++;
        osDelayUntil(tick);
    }
}