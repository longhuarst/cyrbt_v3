//
// Created by Rebecca on 2019/2/16.
//

#include <stm32f103xe.h>
#include <tim.h>
#include <stm32f1xx_hal_tim.h>
#include "cylib_step_motor.h"
#include <math.h>
#include <stdlib.h>

#include "gpio.h"

void cylib_step_motor_io_write(cylib_step_motor_baseio_def io,GPIO_PinState state);
void cylib_step_motor_run_async_motor0(int32_t speed, int32_t step);
void cylib_step_motor_run_async_motor1(int32_t speed, int32_t step);
void cylib_step_motor_run_async_motor2(int32_t speed, int32_t step);







cylib_step_motor_def cylib_step_motor = {
        .instance = {
                {
                        .pin = {
                                .clk    = {.port = STEP_CH1_CLK_GPIO_Port,      .pin = STEP_CH1_CLK_Pin},
                                .dir    = {.port = STEP_CH1_DIR_GPIO_Port,      .pin = STEP_CH1_DIR_Pin},
                                .en     = {.port = STEP_CH1_EN_GPIO_Port,       .pin = STEP_CH1_EN_Pin},
                                .m1     = {.port = STEP_CH1_M1_GPIO_Port,       .pin = STEP_CH1_M1_Pin},
                                .m2     = {.port = STEP_CH1_M2_GPIO_Port,       .pin = STEP_CH1_M2_Pin},
                                .m3     = {.port = STEP_CH1_M3_GPIO_Port,       .pin = STEP_CH1_M3_Pin},
                                .reset  = {.port = STEP_CH1_RESET_GPIO_Port,    .pin = STEP_CH1_RESET_Pin},
                                .st     = {.port = STEP_CH1_ST_GPIO_Port,       .pin = STEP_CH1_ST_Pin},
                        },
                        .state = cylib_step_motor_state_init,
                        .step_sp = 0,
                        .step_cb = 0,
                        .step_error = 0,
                        .htim = &htim1,
                        .step_clk_state = false,
                        .step_forward = true,
                        .step_reduction = 20,
                        .step_degree_sp = 0.0f,
                        .step_degree_cb = 0.0f,
                        .step_degree_error = 0.0f,
                }, {
                        .pin = {
                                .clk    = {.port = STEP_CH2_CLK_GPIO_Port,      .pin = STEP_CH2_CLK_Pin},
                                .dir    = {.port = STEP_CH2_DIR_GPIO_Port,      .pin = STEP_CH2_DIR_Pin},
                                .en     = {.port = STEP_CH2_EN_GPIO_Port,       .pin = STEP_CH2_EN_Pin},
                                .m1     = {.port = STEP_CH2_M1_GPIO_Port,       .pin = STEP_CH2_M1_Pin},
                                .m2     = {.port = STEP_CH2_M2_GPIO_Port,       .pin = STEP_CH2_M2_Pin},
                                .m3     = {.port = STEP_CH2_M3_GPIO_Port,       .pin = STEP_CH2_M3_Pin},
                                .reset  = {.port = STEP_CH2_RESET_GPIO_Port,    .pin = STEP_CH2_RESET_Pin},
                                .st     = {.port = STEP_CH2_ST_GPIO_Port,       .pin = STEP_CH2_ST_Pin},
                        },
                        .state = cylib_step_motor_state_init,
                        .step_sp = 0,
                        .step_cb = 0,
                        .step_error = 0,
                        .htim = &htim7,
                        .step_clk_state = false,
                        .step_forward = true,
                        .step_reduction = 20,
                        .step_degree_sp = 0.0f,
                        .step_degree_cb = 0.0f,
                        .step_degree_error = 0.0f,
                }, {
                        .pin = {
                                .clk    = {.port = STEP_CH3_CLK_GPIO_Port, .pin = STEP_CH3_CLK_Pin},
                                .dir    = {.port = STEP_CH3_DIR_GPIO_Port, .pin = STEP_CH3_DIR_Pin},
                                .en     = {.port = STEP_CH3_EN_GPIO_Port, .pin = STEP_CH3_EN_Pin},
                                .m1     = {.port = STEP_CH3_M1_GPIO_Port, .pin = STEP_CH3_M1_Pin},
                                .m2     = {.port = STEP_CH3_M2_GPIO_Port, .pin = STEP_CH3_M2_Pin},
                                .m3     = {.port = STEP_CH3_M3_GPIO_Port, .pin = STEP_CH3_M3_Pin},
                                .reset  = {.port = STEP_CH3_RESET_GPIO_Port, .pin = STEP_CH3_RESET_Pin},
                                .st     = {.port = STEP_CH3_ST_GPIO_Port, .pin = STEP_CH3_ST_Pin},
                        },
                        .state = cylib_step_motor_state_init,
                        .step_sp = 0,
                        .step_cb = 0,
                        .step_error = 0,
                        .htim = &htim8,
                        .step_clk_state = false,
                        .step_forward = true,
                        .step_reduction = 20,
                        .step_degree_sp = 0.0f,
                        .step_degree_cb = 0.0f,
                        .step_degree_error = 0.0f,
                },
        },
        .step_point_sp = {0.0f, 0.0f, 0.0f,},
        .step_point_cb = {0.0f, 0.0f, 0.0f,},
        .step_point_error = {0.0f, 0.0f, 0.0f,},
};


void cylib_step_motor_init(void)
{
    //拉高所有管脚除了CLK
    for (int i=0; i<3; ++i){
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.clk,GPIO_PIN_RESET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.dir,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.en,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.m1,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.m2,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.m3,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.reset,GPIO_PIN_SET);
        cylib_step_motor_io_write(cylib_step_motor.instance[i].pin.st,GPIO_PIN_SET);
    }
}












void cylib_step_motor_io_write(cylib_step_motor_baseio_def io,GPIO_PinState state)
{
    HAL_GPIO_WritePin(io.port,io.pin,state);
}



void cylib_step_motor_timer_config(TIM_HandleTypeDef *htim, uint32_t speed)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    if (speed == 0 )
        return;



    htim->Init.Prescaler = 71;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 10000/speed;//速度
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    htim->Instance->CNT = 0;
    HAL_TIM_Base_Start_IT(htim);//开始定时器


}



void cylib_step_motor_block_wait_for(int x)
{
    if (x >= 0 && x < 3){
        while(cylib_step_motor.instance[x].state != cylib_step_motor_state_free);
    }

}



void cylib_step_motor_block_wait_for_all(void)
{
    for (int i=0;i<3;++i){
        while(cylib_step_motor.instance[i].state != cylib_step_motor_state_free);
    }

}








//电机同步运动  //需要计算电机的速度
void cylib_step_motor_run_sync(int32_t step[3]){
    if (cylib_step_motor.instance[0].state == cylib_step_motor_state_free &&
            cylib_step_motor.instance[1].state == cylib_step_motor_state_free &&
            cylib_step_motor.instance[2].state == cylib_step_motor_state_free ){


        //归一化 找出最大值

        int32_t max = abs(step[0]);
        if (abs(step[0]) < abs(step[1])) max = abs(step[1]);
        if (max < abs(step[2])) max = abs(step[2]);

        float scale[3];
        uint32_t speed[3];

        for (int i=0; i<3; ++i){
            scale[i] = abs(step[i]) / max;
            speed[i] = 1000 * scale[i];
        }

        cylib_step_motor_run_async_motor0(speed[0], step[0]);
        cylib_step_motor_run_async_motor1(speed[1], step[1]);
        cylib_step_motor_run_async_motor2(speed[2], step[2]);


    }
}

















//电机单独运动 电机0
//参数1 速度  速度等级 1~1000
//参数2 步数 正为正传  负为反转
void cylib_step_motor_run_async_motor0(int32_t speed, int32_t step)
{

    if (cylib_step_motor.instance[0].state != cylib_step_motor_state_free){
        return;
    }

    if (cylib_step_motor.instance[0].step_clk_state !=
        HAL_GPIO_ReadPin(cylib_step_motor.instance[0].pin.clk.port,
                         cylib_step_motor.instance[0].pin.clk.pin)){
        if (cylib_step_motor.instance[0].step_clk_state == false){
            HAL_GPIO_WritePin(cylib_step_motor.instance[0].pin.clk.port,
                              cylib_step_motor.instance[0].pin.clk.pin,
                              GPIO_PIN_RESET);
            cylib_step_motor.instance[0].step_cb++;//增加一步
        }
    }

    cylib_step_motor.instance[0].step_clk_state = false;



    cylib_step_motor.instance[0].step_sp += step;//先设计步数

    cylib_step_motor_timer_config(cylib_step_motor.instance[0].htim,speed);

    cylib_step_motor.instance[0].state = cylib_step_motor_state_running;

}


//电机单独运动 电机1
//参数1 速度 速度等级 1~1000
//参数2 步数 正为正传  负为反转
void cylib_step_motor_run_async_motor1(int32_t speed, int32_t step)
{

    if (cylib_step_motor.instance[1].state != cylib_step_motor_state_free){
        return;
    }

    if (cylib_step_motor.instance[1].step_clk_state !=
        HAL_GPIO_ReadPin(cylib_step_motor.instance[1].pin.clk.port,
                         cylib_step_motor.instance[1].pin.clk.pin)){
        if (cylib_step_motor.instance[1].step_clk_state == false){
            HAL_GPIO_WritePin(cylib_step_motor.instance[1].pin.clk.port,
                              cylib_step_motor.instance[1].pin.clk.pin,
                              GPIO_PIN_RESET);
            cylib_step_motor.instance[1].step_cb++;//增加一步
        }
    }

    cylib_step_motor.instance[1].step_clk_state = false;



    cylib_step_motor.instance[1].step_sp += step;//先设计步数

    cylib_step_motor_timer_config(cylib_step_motor.instance[1].htim,speed);


    cylib_step_motor.instance[1].state = cylib_step_motor_state_running;

}


//电机单独运动 电机2
//参数1 速度 速度等级 1~1000
//参数2 步数 正为正传  负为反转
void cylib_step_motor_run_async_motor2(int32_t speed, int32_t step)
{


    if (cylib_step_motor.instance[2].state != cylib_step_motor_state_free){
        return;
    }

    if (cylib_step_motor.instance[2].step_clk_state !=
        HAL_GPIO_ReadPin(cylib_step_motor.instance[2].pin.clk.port,
                         cylib_step_motor.instance[2].pin.clk.pin)){
        if (cylib_step_motor.instance[2].step_clk_state == false){
            HAL_GPIO_WritePin(cylib_step_motor.instance[2].pin.clk.port,
                              cylib_step_motor.instance[2].pin.clk.pin,
                              GPIO_PIN_RESET);
            cylib_step_motor.instance[2].step_cb++;//增加一步
        }
    }

    cylib_step_motor.instance[2].step_clk_state = false;



    cylib_step_motor.instance[2].step_sp += step;//先设计步数

    cylib_step_motor_timer_config(cylib_step_motor.instance[2].htim,speed);


    cylib_step_motor.instance[2].state = cylib_step_motor_state_running;

}






void cylib_step_motor_timer_callback(int x)
{
    if (x >=0 && x< 3){
        if (cylib_step_motor.instance[x].step_sp == cylib_step_motor.instance[x].step_cb){
            return;
        }

        if (cylib_step_motor.instance[x].step_sp > cylib_step_motor.instance[x].step_cb){
            //正转
            HAL_GPIO_WritePin(
                    cylib_step_motor.instance[x].pin.dir.port,
                    cylib_step_motor.instance[x].pin.dir.pin,
                    GPIO_PIN_SET
            );

            cylib_step_motor.instance[x].step_forward = true;
        }else{
            //反转
            HAL_GPIO_WritePin(
                    cylib_step_motor.instance[x].pin.dir.port,
                    cylib_step_motor.instance[x].pin.dir.pin,
                    GPIO_PIN_RESET
            );

            cylib_step_motor.instance[x].step_forward = false;
        }

        if (cylib_step_motor.instance[x].step_clk_state == false){
            cylib_step_motor.instance[x].step_clk_state = true;
            HAL_GPIO_WritePin(
                    cylib_step_motor.instance[x].pin.clk.port,
                    cylib_step_motor.instance[x].pin.clk.pin,
                    GPIO_PIN_SET
            );


        }else{
            cylib_step_motor.instance[x].step_clk_state = false;
            HAL_GPIO_WritePin(
                    cylib_step_motor.instance[x].pin.clk.port,
                    cylib_step_motor.instance[x].pin.clk.pin,
                    GPIO_PIN_RESET
            );

            if (cylib_step_motor.instance[x].step_forward == true){
                cylib_step_motor.instance[x].step_cb++;
                if (cylib_step_motor.instance[x].step_cb >= cylib_step_motor.instance[x].step_sp){
                    HAL_TIM_Base_Stop_IT(cylib_step_motor.instance[x].htim);
                    cylib_step_motor.instance[x].state = cylib_step_motor_state_free;
                }
            }else{
                cylib_step_motor.instance[x].step_cb--;
                if (cylib_step_motor.instance[x].step_cb <= cylib_step_motor.instance[x].step_sp){
                    HAL_TIM_Base_Stop_IT(cylib_step_motor.instance[x].htim);
                    cylib_step_motor.instance[x].state = cylib_step_motor_state_free;
                }
            }

        }
    }
}



//定时器中断回调函数0
void cylib_step_motor_timer_callback0(void)
{

    cylib_step_motor_timer_callback(0);

}

//定时器中断回调函数1
void cylib_step_motor_timer_callback1(void)
{

    cylib_step_motor_timer_callback(1);

}

//定时器中断回调函数2
void cylib_step_motor_timer_callback2(void)
{

    cylib_step_motor_timer_callback(2);

}




//插补



//开环坐标系统






//角度转步数
//参数1   需要前进的步数
//参数2   减速比
//返回值  步数
int32_t cylib_step_motor_degree_to_step(float degree, uint32_t reduction)
{
    //减速比 reduction
    //细分 128

    int32_t step = 0;

    step = degree / ( 1.8f / 128.0f / reduction );//度 / (度/步) = 步

    return step;

}





//同步运行指定角度
void cylib_step_motor_run_sync_degree(float degree[3])
{

    int32_t step[3];

    step[0] = cylib_step_motor_degree_to_step(degree[0],cylib_step_motor.instance[0].step_reduction);
    step[1] = cylib_step_motor_degree_to_step(degree[1],cylib_step_motor.instance[1].step_reduction);
    step[2] = cylib_step_motor_degree_to_step(degree[2],cylib_step_motor.instance[2].step_reduction);


    cylib_step_motor_run_sync(step);



}



//同步运行指定角度并等待
void cylib_step_motor_run_sync_degree_wait_complete(float degree[3])
{
    cylib_step_motor_run_sync_degree(degree);
    cylib_step_motor_block_wait_for_all();
}





void cylib_step_motor_oxy_to_degrees(float point[3], float degree[3])
{
    //此处添加各种机械臂的坐标转换公式

    //FIX ME !

}




void cylib_step_motor_run_point(float point[3])
{
    float degree[3];
    float degree_error[3];

    cylib_step_motor_oxy_to_degrees(point, degree);


    //得到的角度
    cylib_step_motor.instance[0].step_degree_sp = degree[0];
    cylib_step_motor.instance[1].step_degree_sp = degree[1];
    cylib_step_motor.instance[2].step_degree_sp = degree[2];


    degree_error[0] = cylib_step_motor.instance[0].step_degree_error = cylib_step_motor.instance[0].step_degree_sp -
                                                     cylib_step_motor.instance[0].step_degree_cb;

    degree_error[1] = cylib_step_motor.instance[1].step_degree_error = cylib_step_motor.instance[1].step_degree_sp -
                                                     cylib_step_motor.instance[1].step_degree_cb;

    degree_error[2] = cylib_step_motor.instance[2].step_degree_error = cylib_step_motor.instance[2].step_degree_sp -
                                                     cylib_step_motor.instance[2].step_degree_cb;

    //误差角度就是需要走的角度
    cylib_step_motor_run_sync_degree_wait_complete(degree_error);


    cylib_step_motor.instance[0].step_degree_cb = cylib_step_motor.instance[0].step_degree_sp;
    cylib_step_motor.instance[1].step_degree_cb = cylib_step_motor.instance[1].step_degree_sp;
    cylib_step_motor.instance[2].step_degree_cb = cylib_step_motor.instance[2].step_degree_sp;
}





//插补走直线
//参数1 目标坐标
//参数2 速度(1~100)
void cylib_step_motor_run_point_runin(float point[3],uint32_t speed)
{

    if (speed < 1 || speed > 1000)
        return;

    for (int j=0; j<3; ++j){
        cylib_step_motor.step_point_sp[j] = point[j];
        cylib_step_motor.step_point_error[j] = cylib_step_motor.step_point_sp[j] - cylib_step_motor.step_point_cb[j];
    }

    //固定插补
    #define runin_limit (0.1f)
    uint32_t times = 100*1000 / speed;

    for (int i=0;i<times;++i){
        float next[3];
        next[0] = cylib_step_motor.step_point_error[0] / times * (i+1) + cylib_step_motor.step_point_cb[0];
        next[1] = cylib_step_motor.step_point_error[1] / times * (i+1) + cylib_step_motor.step_point_cb[1];
        next[2] = cylib_step_motor.step_point_error[2] / times * (i+1) + cylib_step_motor.step_point_cb[2];

        cylib_step_motor_run_point(next);

        cylib_step_motor.step_point_cb[0] = next[0];
        cylib_step_motor.step_point_cb[1] = next[1];
        cylib_step_motor.step_point_cb[2] = next[2];


    }

}


//电机校准
void cylib_step_motor_calibrate(void)
{
    //此处添加各种机械臂的电机校准程序

    //FIX ME !

}


























