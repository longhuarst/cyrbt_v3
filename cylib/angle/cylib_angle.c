#include "cylib_angle.h"

#include <stdbool.h>
#include <string.h>






cylib_angle_def cylib_angle[4];


void cylib_angle_init(void)
{
	
	memset(&cylib_angle[0],0,sizeof(cylib_angle_def));
	memset(&cylib_angle[1],0,sizeof(cylib_angle_def));
	memset(&cylib_angle[2],0,sizeof(cylib_angle_def));
	memset(&cylib_angle[3],0,sizeof(cylib_angle_def));
}



void cylib_angle_update(int index, float x, float y)
{
	if (index <0 || index > 3)
		return;
	
	cylib_angle[index].x = x;
	cylib_angle[index].y = y;
	cylib_angle[index].updated = true;
	
	
}



uint8_t cylib_angle_wait_init(void)
{
	uint8_t tmp = 0;
	
	for (int i=0;i<4;++i){
		if (cylib_angle[i].updated){
			tmp |= 0x1u<<i;
		}
	}
	
	return tmp;
	
	
}


bool cylib_angle_get(int x, float *angle)
{
	if (x <0 || x>= 4){
		return false;
	}
	
	if (cylib_angle[x].updated == false)
		return false;
	
	*angle = cylib_angle[x].y;
	
	cylib_angle[x].updated = false;
	
	return true;
	
}

