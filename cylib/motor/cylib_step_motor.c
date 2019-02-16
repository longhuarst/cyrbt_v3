//
// Created by Rebecca on 2019/2/16.
//

#include "cylib_step_motor.h"


#include "gpio.h"

void cylib_step_motor_io_write(cylib_step_motor_baseio_def io,GPIO_PinState state);








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
                },
        },
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





//电机同步运动
void cylib_step_motor_run_sync(){

}



//定时器中断回调函数0
void cylib_step_motor_timer_callback0(void)
{


}

//定时器中断回调函数1
void cylib_step_motor_timer_callback1(void)
{


}

//定时器中断回调函数2
void cylib_step_motor_timer_callback2(void)
{


}




//插补



//开环坐标系统




