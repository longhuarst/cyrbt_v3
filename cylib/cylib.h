//
// Created by Rebecca on 2019/2/16.
//

#ifndef CYRBT_V3_CYLIB_H
#define CYRBT_V3_CYLIB_H



#include "motor/cylib_step_motor.h"
#include "serial/cylib_serial.h"
#include "gcoder/cylib_gcoder.h"
#include "can/cylib_can.h"
#include "angle/cylib_angle.h"
#include "switch/cylib_switch.h"
#include "abb/cylib_abb.h"
#include "location/cylib_location.h"
#include "controller/cylib_controller.h"


#define mBYTE0(dwTemp)       (*(char *)(&dwTemp))
#define mBYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define mBYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define mBYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))




extern void cylib_init(void);






#endif //CYRBT_V3_CYLIB_H
