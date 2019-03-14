//
// Created by Rebecca on 2019/2/16.
//

#include "cylib.h"






void cylib_init(void)
{

    cylib_serial_init();//初始化串口

    cylib_step_motor_init();//初始化步进电机

    cylib_gcoder_init();//G代码初始化

	cylib_angle_init();//角度初始化
	
	cylib_can_init();//CAN初始化
	
	cylib_switch_init();//限位开关
	
	cylib_abb_init();
	
	
	cylib_location_init();//初始化位置坐标
	
	
	cylib_controller_init();//初始化控制器
}






