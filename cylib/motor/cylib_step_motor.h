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

        int32_t step_sp;//目标位置
        int32_t step_cb;//反馈位置
        int32_t step_error;//位置误差
        bool step_clk_state;//当前CLK的状态
        bool step_forward;//电机正转
        uint32_t step_reduction;//电机的减速比

        float step_degree_sp;//当前的角度
        float step_degree_cb;//反馈的角度
        float step_degree_error;//当前的误差角度



        TIM_HandleTypeDef *htim;//占用定时器

    }instance[3];




    float step_point_sp[3];//当前的目标坐标
    float step_point_cb[3];//反馈的坐标
    float step_point_error[3];//当前坐标误差


}cylib_step_motor_def;






extern cylib_step_motor_def cylib_step_motor;

extern void cylib_step_motor_init(void);

extern void cylib_step_motor_timer_callback0(void);
extern void cylib_step_motor_timer_callback1(void);
extern void cylib_step_motor_timer_callback2(void);

void cylib_step_motor_run_async_motor0(int32_t speed, int32_t step);
void cylib_step_motor_run_async_motor1(int32_t speed, int32_t step);
void cylib_step_motor_run_async_motor2(int32_t speed, int32_t step);


void cylib_step_motor_block_wait_for_all(void);

//插补走直线
//参数1 目标坐标
//参数2 速度(1~100)
extern void cylib_step_motor_run_point_runin(float point[3],uint32_t speed);



#endif //CYRBT_V3_CYLIB_STEP_MOTOR_H
