#include "b.h"

#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../motor/cylib_step_motor.h"

#define r1 (120)
#define r2 (128.5)

#define C_PI       3.14159265358979323846
#define Rad2Deg(x) (180 / C_PI * x)


extern void user_delay_us(uint32_t delay);

void cylib_b_init(void)
{

}



void cylib_b_calc(float x, float y, float z, float *angle0, float *angle2)
{
	float R = sqrt(x*x+y*y);
	float dg2;
	float dgx = 0;
	float dg1 =0;
	//angle0 ���
	//angle2 �ڽ�

	
	if(y <0 )
		return;
		
	//����XY y>0
	
//	if (y <0 )
//		return;//ֻ������
//	
//	if(R = r1 + r2){
//		//��Բ��
//		*angle0 = 0;
//		
//	}
//	
	
	if (R == r1+r2){
		*angle0 = 0;
		if (x ==0 )
			*angle2 = 90;
		else {
			if (x > 0){
				
				dgx = Rad2Deg(atan2(y,x));
				*angle2 = dgx;
			}else{
				//x<0
				dgx = Rad2Deg(atan2(y,x));
				*angle2 = dgx;
			}
		}
	}else if (R < r1 + r2){
		dg2 = 180 - Rad2Deg((acos((r1*r1 + r2 * r2 - R * R) / (2 * r1*r2))));
		*angle0 = dg2;
		
		dgx = Rad2Deg(acos((r1*r1+R*R-r2*r2)/(2*r1*R)));
		
		if (x == 0){
			*angle2 = dgx + 90;
		}else{
			float dg3 = Rad2Deg(atan2(y,x));
			*angle2 = dg3 + dgx;
		}
	}else{
		//��̎��
	}
	
	

//	

//	if (R < r1 + r2) {
//	
//		dg2 = 180 - Rad2Deg((acos((r1*r1 + r2 * r2 - R * R) / (2 * r1*r2))));

//		dgx = Rad2Deg(acos((r1*r1+R*R-r2*r2)/(2*r1*R)));

//		if (x == 0) {
//			if (y > 0) {
//				dg1 = 360 - dgx;
//			}
//			else if (y < 0){
//				dg1 = 180 - dgx;
//			}
//			else if (y == 0) {
//				//printf("���꣨x,y�� = (0,0) ���ɴ\r\n");
//				return;
//			}
//		}
//		else if (x > 0) {
//			if (y == 0) {
//				dg1 = 90 - dgx;
//			}
//			else if (y > 0) {
//				dg1 = 90 - dgx - Rad2Deg(atan(fabs(y / x)));
//			}
//			else if (y < 0) {
//				dg1 = 180 - dgx - Rad2Deg(atan(fabs(x / y)));
//				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(x / y))));
//			}
//		}
//		else if (x < 0) {
//			if (y == 0) {
//				dg1 = 270 - dgx;
//			}
//			else if (y > 0) {
//				dg1 = 360 - dgx - Rad2Deg(atan(fabs(x/y)));
//				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(x / y))));
//			}
//			else if (y < 0) {
//				dg1 = 270 - dgx - Rad2Deg(atan(fabs(y / x)));
//				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(y / x))));
//			}
//		}
//	}
//	else if (R == r1 + r2) {
//		if (x == 0) {
//			if (y > 0) {
//				dg1 = 0;
//				dg2 = 0;
//			}
//			else if (y < 0) {
//				dg1 = 180;
//				dg2 = 0;
//			}
//			else {
//				//printf("���꣨x,y�� = (0,0) ���ɴ\r\n");
//				return;
//			}
//		}
//		else if (x > 0) {
//			if (y > 0) {
//				dg1 =  90 - Rad2Deg(atan(fabs(y/x)));
//				dg2 = 0;
//			}
//			else if (y < 0) {
//				dg1 = 90 + Rad2Deg(atan(fabs(y / x)));
//				dg2 = 0;
//			}
//			else {
//				dg1 = 90;
//				dg2 = 0;
//			}

//		}
//		else if (x < 0) {
//			if (y > 0) {
//				dg1 = 270 + Rad2Deg(atan(fabs(y / x)));
//				dg2 = 0;
//			}
//			else if (y < 0) {
//				dg1 = 270 - Rad2Deg(atan(fabs(y / x)));
//				dg2 = 0;
//			}
//			else {
//				dg1 = 270;
//				dg2 = 0;
//			}
//		}
//	}
//	else if (R > r1 + r2) {
//		//printf("���볬�����㷶Χ!\r\n");
//	}


//	
//	*angle0 = dg2;//����
//	*angle1 = dg1;//���ĵ��
//	
//	if (*angle1 > 180)
//		*angle1 = *angle1 - 360;

	//printf("(%f,%f) --> (%f,%f)\r\n",x,y,dg1,dgx);
	
	
}


float cylib_b_degree_holding_dg1 = 0.0f;//����
float cylib_b_degree_holding_dg2 = 90.0f;//�ڵ��
float cylib_b_degree_holding_dg3 = 0.0f;//�߶ȵ��

void cylib_b_move(float x, float y, float z)
{
	float g1,g2;
	
	
	cylib_b_calc(x,y,z,&g1,&g2);
	
	
	
	//�˶� ����
	{
		float error = (g1 - cylib_b_degree_holding_dg1);
		//��ת��
		
		int32_t step = 711.1111111111111f * error;
		
		if (step > 0){
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_SET);
		}else{
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_RESET);
		}
		
		
		for (int i=0;i<abs(step);++i){
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_SET);
			user_delay_us(100);
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_RESET);
			user_delay_us(100);
		}
		
		cylib_b_degree_holding_dg1 = g1;
	}
	
	
	//�˶� �ڵ��
	{
		float error = (g2 - cylib_b_degree_holding_dg2);
		//��ת��
		
		int32_t step = 711.1111111111111f * error;
		
		if (step > 0){
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.dir,GPIO_PIN_SET);
		}else{
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.dir,GPIO_PIN_RESET);
		}
		
		
		for (int i=0;i<abs(step);++i){
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.clk,GPIO_PIN_SET);
			user_delay_us(100);
			cylib_step_motor_io_write(cylib_step_motor.instance[2].pin.clk,GPIO_PIN_RESET);
			user_delay_us(100);
		}
		
		cylib_b_degree_holding_dg2 = g2;
	}
	
	
	
	//z��  У׼�߶�Ϊ0  ֻ��Ϊ��ֵ
	
	{
		float error = z - cylib_b_degree_holding_dg3;
		if (z > 0){
			
		}else{
			
			#define MM_TO_STEP(X) (((X)/8.0f*360/(1.8/128)))
			
			int32_t step = MM_TO_STEP(error);
		
			if (step > 0){
				cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_SET);
			}else{
				cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_RESET);
			}
			
			
			for (int i=0;i<abs(step);++i){
				cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_SET);
				user_delay_us(100);
				cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_RESET);
				user_delay_us(100);
			}
			
			cylib_b_degree_holding_dg3 = z;
		}
		
		
	}
	
	
	
}
