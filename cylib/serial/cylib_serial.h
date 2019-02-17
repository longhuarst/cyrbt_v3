//
// Created by Rebecca on 2019/2/17.
//

#ifndef CYRBT_V3_CYLIB_SERIAL_H
#define CYRBT_V3_CYLIB_SERIAL_H


#include <stdint.h>




extern void cylib_serial_init(void);


extern void cylib_serial_rx_callback(int x);



extern int32_t cylib_serial_read(int32_t fd, uint8_t *buffer, int32_t length);



#endif //CYRBT_V3_CYLIB_SERIAL_H
