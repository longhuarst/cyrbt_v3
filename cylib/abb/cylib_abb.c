#include "cylib_abb.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>


void cylib_abb_init(void)
{
	
}





#define C_PI       3.14159265358979323846
#define Rad2Deg(x) (180 / C_PI * x)

//#define L (160)
//#define L3 (59)
//#define H (131)
//#define H2 (0)

#define C_Motor_Step_Deg (1.8/128/10) //每个脉冲的角度
#define C2_Motor_Step_Deg (1.8/128/30) //每个脉冲的角度

//													&indg, &outdg, &yawdg
void cylib_abb_calc(float x, float y, float z, float *angle0, float *angle1, float *angle2)
{
	
	printf("x = %f, y = %f, z = %f \r\n",x,y,z);
	float R = sqrt(x*x+y*y);
	
	#define Z_C (131)
	
	if (x == 0 && y == 0){
		//无解
	}else if (x == 0){
		if (y > 0){
			*angle2 = 0;
		}else if (y < 0){
			*angle2 = 180;
		}
	}else if (y == 0){
		if (x > 0){
			*angle2 = 90;
		}else if (x < 0){
			*angle2 = -90;
		}
	}else{
		
		if (x > 0 && y > 0){
			//第一象限
			*angle2 = atan(x/y);
		}else if (x > 0 && y < 0){
			//第四象限
			*angle2 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//第二象限
			*angle2 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//第三象限
			*angle2 = -atan(fabs(y/x)) - 90;
		}else{
			//计算无效
		}
		*angle2 = Rad2Deg(*angle2);
	}
//	
//	
//	
//	
//	
//	float a = (R - L3) / L;
//	
//	float b = (z - H - H2) / L;
//	
//	
//	*angle0 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
//	
//	
//	*angle1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
//	
//	
//	*angle0 = Rad2Deg(*angle0);
//	
//	*angle1 = - Rad2Deg(*angle1);//d2 需要取负角
	
	
	
	//float R = sqrt(x*x+y*y);
	
	if (z == Z_C) {
	
		#define a (160)
		#define b (R)
		#define c (160)
		
		
		float deg_c = Rad2Deg(acosf((a*a + b*b - c*c) / (2 * a * b)));
		
		float deg_b = Rad2Deg(acosf((a*a + c*c - b*b) / (2 * a * c)));
		
		float in_deg,out_deg;
		
		in_deg = deg_c;
		
		float deg_d = - (180 - deg_c - deg_b );
		
		*angle0 = deg_c;
		*angle1 = deg_d;
		
		return;
	}
	
	
	
	if (z > Z_C) {
		float AD = R;
		float L = z - Z_C;
		float DE = L;
		float AE = sqrt(AD*AD+DE*DE);
		#define AB (160)
		#define BE (160)
		float deg_B = Rad2Deg(acosf((AB*AB + BE*BE - AE*AE) / (2 * AB * BE)));
		float AF = L;
		#define FE (R)
		float deg_x = Rad2Deg(atan2(AF,FE));
		float deg_BEA = Rad2Deg(acosf((BE*BE + AE*AE - AB*AB) / (2 * BE * AE)));
		float deg_BEF = deg_x + deg_BEA;
		float deg_GBE = deg_BEF;
		float deg_ABE = Rad2Deg(acosf((AB*AB + BE*BE - AE*AE) / (2 * AB * BE)));
		float deg_HBA = - (180 - deg_ABE - deg_GBE);
		
		*angle0 = deg_BEF;
		*angle1 = deg_HBA;
		
		return;

	}
	
	
	if (z < Z_C){
		#define AB (160)
		#define BC (160)
		#define AD (R)
		
		float L = Z_C - z;
		float CD = L;
		
		float L1;
		float AC = sqrt(AD*AD + CD*CD);
		float deg_C = Rad2Deg(acosf((BC*BC + AC*AC - AB*AB) / (2 * BC * AC)));
		float deg_x = Rad2Deg(atan2(R,L));
		float deg_C_Level = (deg_x + deg_C ) - 90;
		float deg_B = Rad2Deg(acosf((AB*AB + BC*BC - AC*AC) / (2 * AB * BC)));
		float deg_y = - (180 - deg_B - deg_C_Level);
		
		*angle0 = deg_C_Level;
		*angle1 = deg_y;
		
		return;
		
	}
	
}



