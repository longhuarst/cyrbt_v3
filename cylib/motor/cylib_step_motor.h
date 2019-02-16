//
// Created by Rebecca on 2019/2/16.
//

#ifndef CYRBT_V3_CYLIB_STEP_MOTOR_H
#define CYRBT_V3_CYLIB_STEP_MOTOR_H


#include "gpio.h"
#include <stdbool.h>



typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
}cylib_step_motor_baseio_def;

typedef enum {
    cylib_step_motor_state_init = 0, //初始化状态
    cylib_step_motor_state_running, //运动
    cylib_step_motor_state_lock,//锁定
    cylib_step_motor_state_free, //空闲
}cylib_step_motor_state;

typedef struct{
    struct {

        struct {
            cylib_step_motor_baseio_def clk;
            cylib_step_motor_baseio_def dir;
            cylib_step_motor_baseio_def en;
            cylib_step_motor_baseio_def m1;
            cylib_step_motor_baseio_def m2;
            cylib_step_motor_baseio_def m3;
            cylib_step_motor_baseio_def reset;
            cylib_step_motor_baseio_def st;
        }pin;
        cylib_step_motor_state state;//电机目前的状态
    }instance[3];


}cylib_step_motor_def;


extern cylib_step_motor_def cylib_step_motor;

extern void cylib_step_motor_init(void);

extern void cylib_step_motor_timer_callback0(void);
extern void cylib_step_motor_timer_callback1(void);
extern void cylib_step_motor_timer_callback2(void);






#endif //CYRBT_V3_CYLIB_STEP_MOTOR_H
