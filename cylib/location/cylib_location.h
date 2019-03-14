



#ifndef CYRBT_V3_CYLIB_LOCATION_H
#define CYRBT_V3_CYLIB_LOCATION_H


#include <stdint.h>
#include <stdbool.h>



typedef struct{
	struct{
		float x;
		float y;
		float z;
	}instance[4];
}cylib_location_def;


extern cylib_location_def cylib_location;

extern void cylib_location_init(void);


extern void cylib_location_get(int index, float *x, float *y, float *z);
extern void cylib_location_set(int index, float x, float y, float z);


#endif //CYRBT_V3_CYLIB_LOCATION_H
