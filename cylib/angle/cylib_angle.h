



#ifndef CYRBT_V3_CYLIB_ANGLE_H
#define CYRBT_V3_CYLIB_ANGLE_H


#include <stdint.h>
#include <stdbool.h>
typedef struct{
	float x;
	float y;
	bool updated;
}cylib_angle_def;


extern cylib_angle_def cylib_angle[4];


extern void cylib_angle_init(void);
extern void cylib_angle_update(int index, float x, float y);
extern uint8_t cylib_angle_wait_init(void);
extern bool cylib_angle_get(int x, float *angle);





#endif //CYRBT_V3_CYLIB_ANGLE_H
