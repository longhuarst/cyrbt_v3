#include "delta.h"

#include <math.h>

extern void user_delay_us(uint32_t delay);

//统一单位 mm

#define C_PI       3.14159265358979323846
#define Rad2Deg(x) (180/C_PI * x)
#define C_La (220)
#define C_Lb (110)
#define C_R (86.603f)
#define C_r (47.962f)

#define C_X0Y0Z0_DG1 (80) //初始化电机角度
#define C_Motor_Step_Deg (1.8/128/10) //每个脉冲的角度

float c_x =0,c_y = 0,c_z = -300;//坐标

float c_k1,c_u1,c_v1,c_k2,c_u2,c_v2,c_k3,c_u3,c_v3;//计算参数

float c_t1_1,c_t1_2;
float c_t2_1,c_t2_2;
float c_t3_1,c_t3_2;

float c_dg_1_1,c_dg_1_2;
float c_dg_2_1,c_dg_2_2;
float c_dg_3_1,c_dg_3_2;


float current_deg1 = C_X0Y0Z0_DG1;
float current_deg2 = C_X0Y0Z0_DG1;
float current_deg3 = C_X0Y0Z0_DG1;

 
void location_calc()
{

	c_k1 = (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) + (C_R-C_r)*(1.7321f*c_x+c_y)) /(C_Lb) + 2*c_z;
	c_u1 = -2 * (2*(C_R-C_r) - 1.7321*c_x - c_y);
	c_v1 =  (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) + (C_R-C_r)*(1.7321*c_x+c_y)) / C_Lb - 2*c_z;
														
	c_k2 = (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) - (C_R-C_r)*(1.7321*c_x-c_y)) / C_Lb + 2*c_z;
	c_u2 = -2 * (2*(C_R-C_r) + 1.7321*c_x - c_y);
	c_v2 = (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) - (C_R-C_r)*(1.7321*c_x-c_y)) / C_Lb - 2*c_z;
															
	c_k3 = (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) - 2*c_y*(C_R-C_r)) / (2*C_Lb) + c_z;
	c_u3 = -2 * (C_R - C_r + c_y);
	c_v3 = (C_La*C_La - C_Lb*C_Lb - c_x*c_x -c_y*c_y - c_z*c_z - (C_R-C_r)*(C_R-C_r) - 2*c_y*(C_R-C_r)) / (2*C_Lb) - c_z;
				

	//c_t1_1 = (-c_u1 + sqrt(c_u1*c_u1 - 4*c_k1*c_v1));// / (2*c_k1);
	c_t1_2 = (-c_u1 - sqrt(c_u1*c_u1 - 4*c_k1*c_v1));// / (2*c_k1);
	
	//c_t2_1 = (-c_u2 + sqrt(c_u2*c_u2 - 4*c_k2*c_v2));// / (2*c_k2);
	c_t2_2 = (-c_u2 - sqrt(c_u2*c_u2 - 4*c_k2*c_v2));// / (2*c_k2);
	
	//c_t3_1 = (-c_u3 + sqrt(c_u3*c_u3 - 4*c_k3*c_v3));// / (2*c_k3);
	c_t3_2 = (-c_u3 - sqrt(c_u3*c_u3 - 4*c_k3*c_v3));// / (2*c_k3);
	
	
	//c_dg_1_1 = Rad2Deg(2 * atan2(c_t1_1,(2*c_k1))) + 360;
	c_dg_1_2 = Rad2Deg(2 * atan(c_t1_2/(2*c_k1)));// + 360;
	
	//c_dg_2_1 = Rad2Deg(2 * atan2(c_t2_1,(2*c_k2))) + 360;
	c_dg_2_2 = Rad2Deg(2 * atan(c_t2_2/(2*c_k2)));// + 360;
	
	//c_dg_3_1 = Rad2Deg(2 * atan2(c_t3_1,(2*c_k3))) + 360;
	c_dg_3_2 = Rad2Deg(2 * atan(c_t3_2/(2*c_k3)));// + 360;
	
	
	
	//取第一组值 c_dg_1_1 c_dg_2_1 c_dg_3_1	
	
	//计算角度差
	
	
	float d1 = c_dg_1_2 - current_deg1;
	float d2 = c_dg_2_2 - current_deg2;
	float d3 = c_dg_3_2 - current_deg3;


	current_deg1 = c_dg_1_2;
	current_deg2 = c_dg_2_2;
	current_deg3 = c_dg_3_2;

	//计算步数
	int32_t step1 =  (d1/C_Motor_Step_Deg);
	int32_t step2 =  (d2/C_Motor_Step_Deg);
	int32_t step3 =  (d3/C_Motor_Step_Deg);

//	motor[motor_1].step_ = step1;
//	motor[motor_2].step_ = step2;
//	motor[motor_3].step_ = step3;

	for (int i=0;i<abs(step1); ++i){
		if (step1 > 0){
			//set
			cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_SET);
		}else{
			//reset
			cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_RESET);
		} 
		
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_SET);
		user_delay_us(100);
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_RESET);
		user_delay_us(100);
	}
	
	for (int i=0;i<abs(step2); ++i){
		if (step2 > 0){
			//set
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_SET);
		}else{
			//reset
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_RESET);
		} 
		
		cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_SET);
		user_delay_us(100);
		cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_RESET);
		user_delay_us(100);
	}
	
	for (int i=0;i<abs(step3); ++i){
		if (step3 > 0){
			//set
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.dir,GPIO_PIN_SET);
		}else{
			//reset
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.dir,GPIO_PIN_RESET);
		} 
		
		cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.clk,GPIO_PIN_SET);
		user_delay_us(100);
		cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.clk,GPIO_PIN_RESET);
		user_delay_us(100);
	}

}

 
 

void cylib_delta_init(void)
{
	
}


void cylib_delta_calc(float x, float y, float z, float *angle0, float *angle1, float *angle2)
{

}

void cylib_delta_move(float x, float y, float z)
{
	c_x = x;
	c_y = y;
	c_z = z;
	
	
	location_calc();

}