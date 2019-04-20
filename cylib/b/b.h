#ifndef __b_h
#define __b_h

#include <stdint.h>
#include <stdbool.h>




extern void cylib_b_init(void);
extern void cylib_b_calc(float x, float y, float z, float *angle0, float *angle1);


extern void cylib_b_move(float x, float y, float z);


#endif

