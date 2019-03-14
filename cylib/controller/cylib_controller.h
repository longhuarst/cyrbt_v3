



#ifndef CYRBT_V3_CYLIB_CONTROLLER_H
#define CYRBT_V3_CYLIB_CONTROLLER_H


#include <stdint.h>
#include <stdbool.h>



typedef struct{
	struct{
		float x;
		float y;
		float z;
	}instance[4];
}cylib_controller_def;


extern cylib_controller_def cylib_controller;

extern void cylib_controller_init(void);

extern void cylib_controller_move(float x, float y, float z, int speed);


#endif //CYRBT_V3_CYLIB_CONTROLLER_H
