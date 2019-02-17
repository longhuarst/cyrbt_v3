//
// Created by Rebecca on 2019/2/16.
//

#include "cylib.h"






void cylib_init(void)
{

    cylib_serial_init();//初始化串口

    cylib_step_motor_init();//初始化步进电机

    cylib_gcoder_init();//G代码初始化

}






