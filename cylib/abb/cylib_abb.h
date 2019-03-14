



#ifndef CYRBT_V3_CYLIB_ABB_H
#define CYRBT_V3_CYLIB_ABB_H


#include <stdint.h>
#include <stdbool.h>




extern void cylib_abb_init(void);
extern void cylib_abb_calc(float x, float y, float z, float *angle0, float *angle1, float *angle2);



#endif //CYRBT_V3_CYLIB_ABB_H
