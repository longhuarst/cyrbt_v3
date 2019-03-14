#include "cylib_location.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>






cylib_location_def cylib_location;



void cylib_location_init(void)
{
	memset(&cylib_location,0,sizeof(cylib_location));
}


void cylib_location_get(int index, float *x, float *y, float *z)
{
	if (index < 0 || index >= 1){
		return;
	}
	
	*x = cylib_location.instance[index].x;
	*y = cylib_location.instance[index].y;
	*z = cylib_location.instance[index].z;
}


void cylib_location_set(int index, float x, float y, float z)
{
	if (index < 0 || index >= 1){
		return;
	}
	
	cylib_location.instance[index].x = x;
	cylib_location.instance[index].y = y;
	cylib_location.instance[index].z = z;
}
















