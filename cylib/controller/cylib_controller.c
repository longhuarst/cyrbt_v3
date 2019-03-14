#include "cylib_controller.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "../abb/cylib_abb.h"

#include "../angle/cylib_angle.h"


#include "../motor/cylib_step_motor.h"
#include "../location/cylib_location.h"


cylib_controller_def cylib_controller;





void cylib_controller_init(void)
{
	
}

void cylib_controller_move(float x, float y, float z, int speed)
{
	static float angle_history = 0.0f;
	
	float outdg, indg, yawdg;
	cylib_abb_calc(x, y, z, &indg, &outdg, &yawdg);

	
	printf("calc degree : in= %f, out= %f, yaw= %f", indg, outdg, yawdg);
	
	
	//cylib_step_motor_run_async_motor2(100,-128000*1.2); //水平
	
	//水平电机
	{
		float angle = angle_history;
		
		printf("\r\n\r\n----------\r\n");
		printf("yaw current deg = %f , sp deg = %f\r\n",angle, yawdg);
		printf("----------\r\n");
		
		int32_t step = (yawdg - angle) / 0.00140625f;
		
		
		cylib_step_motor_run_async_motor2(100,step); //水平
		
		cylib_step_motor_block_wait_for_all();//等待电机完成
		
		angle_history = yawdg;
	}
	
	//内电机
	{
		//读取惯导0数据
		float angle = 0.0f;
		while (cylib_angle_get(0,&angle) == false);
		
		printf("\r\n\r\n----------\r\n");
		printf("current deg = %f , sp deg = %f\r\n",angle, indg);
		printf("----------\r\n");
		
		
		int32_t step = (indg - angle ) / 0.00046875f;
		
		cylib_step_motor_run_async_motor1(100,step);
		cylib_step_motor_block_wait_for_all();//等待电机完成
	}
	
	
	//外电机
	{
		//读取惯导0数据
		float angle = 0.0f;
		while (cylib_angle_get(1,&angle) == false);
		
		printf("\r\n\r\n----------\r\n");
		printf("current deg = %f , sp deg = %f\r\n",angle, outdg);
		printf("----------\r\n");
		
		//电机2 角度为 45度
		int32_t step = -(outdg - angle ) / 0.00046875f;
		
		cylib_step_motor_run_async_motor0(100,step);
		cylib_step_motor_block_wait_for_all();//等待电机完成
	}
	
	
	
	cylib_location_set(0,x,y,z);//设置坐标点
	
}


