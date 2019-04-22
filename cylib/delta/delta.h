#ifndef __delta_h
#define __delta_h



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../motor/cylib_step_motor.h"




extern void cylib_delta_init(void);
extern void cylib_delta_calc(float x, float y, float z, float *angle0, float *angle1, float *angle2);


extern void cylib_delta_move(float x, float y, float z);


#endif

