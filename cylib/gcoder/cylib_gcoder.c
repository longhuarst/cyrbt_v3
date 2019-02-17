//
// Created by Rebecca on 2019/2/17.
//

#include "cylib_gcoder.h"

#include "../motor/cylib_step_motor.h"
#include <stdint.h>
#include "../serial/cylib_serial.h"
#include <stdio.h>
#include <string.h>

#define cylib_gcoder_uart (0)


typedef struct {
    uint8_t buffer[128];
    uint8_t index;

}cylib_gcoder_def;


cylib_gcoder_def cylib_gcoder = {
        .buffer = {0,},
        .index = 0

};


void cylib_gcoder_init(void)
{





}



void cylib_gcoder_low_to_high(uint8_t *buffer)
{

    if (buffer == 0)
        return;

    int len = strlen(buffer);

    for (int i=0; i<len; ++i){
        if (buffer[i] >= 'a' && buffer[i] <= 'z'){
            buffer[i] = buffer[i] - 'a' + 'A';
        }
    }
}

void cylib_gcoder_decoder_g(void)
{

    uint8_t code = (cylib_gcoder.buffer[1] - '0') * 10 + (cylib_gcoder.buffer[2] - '0');

    float point[3];
    int rescnt;

    cylib_gcoder_low_to_high(cylib_gcoder.buffer);//小写转大写

    switch (code){
        case 0://快速定位 G00 X_ Y_ Z_
            rescnt = sscanf(cylib_gcoder.buffer,"GOO X%f Y%f Z%f",&point[0],&point[1],&point[2]);
            if (rescnt == 3){
                //解码成功了
                cylib_step_motor_run_point_runin(point,100);
            }
            break;
        case 1:

            break;
        case 4:

            break;
        case 90:

            break;
        case 91:

            break;
        default:
            break;
    }
}



void cylib_gcoder_decoder_m(void)
{
    uint8_t code = (cylib_gcoder.buffer[1] - '0') * 10 + (cylib_gcoder.buffer[2] - '0');


    switch (code){
        case 2:

            break;
        case 3:

            break;
        case 4:

            break;
        case 5:

            break;
        default:
            break;
    }

}



void cylib_gcoder_decoder(void)
{

    int32_t res = cylib_serial_read(cylib_gcoder_uart,&cylib_gcoder.buffer[cylib_gcoder.index],1);

    if (res <= 0){
        return;
    }

    cylib_gcoder.index++;


    if (cylib_gcoder.buffer[cylib_gcoder.index-1] == '\r' ||
            cylib_gcoder.buffer[cylib_gcoder.index-1] == '\n'){

        cylib_gcoder.buffer[cylib_gcoder.index-1] = 0;//字符串结束

        //解码

        switch (cylib_gcoder.buffer[0]){
            case 'G':
                cylib_gcoder_decoder_g();
                break;
            case 'M':
                cylib_gcoder_decoder_m();
                break;
            default:
                break;
        }




        cylib_gcoder.index = 0;

    }




    if (cylib_gcoder.index >= 127){
        cylib_gcoder.index = 0;
    }




}




void cylib_gcoder_polling(void)
{

    cylib_gcoder_decoder();

}