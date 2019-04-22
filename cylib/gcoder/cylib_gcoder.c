//
// Created by Rebecca on 2019/2/17.
//

#include "cylib_gcoder.h"

#include "../motor/cylib_step_motor.h"
#include <stdint.h>
#include "../serial/cylib_serial.h"
#include <stdio.h>
#include <string.h>

#include "../controller/cylib_controller.h"
#include "../b/b.h"
#include "../delta/delta.h"

#define cylib_gcoder_uart (0)

int cylib_gcoder_type = -1;

typedef struct {
	uint8_t bufferx[32];
    uint8_t buffer[128];
    uint8_t index;
	bool isInited;
}cylib_gcoder_def;


cylib_gcoder_def cylib_gcoder = {
        .buffer = {0,},
        .index = 0,
		.isInited = false,

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

    static float point[3];
    int rescnt;
	float delay;
	
    cylib_gcoder_low_to_high(cylib_gcoder.buffer);//小写转大写

    switch (code){
        case 0://快速定位 G00 X_ Y_ Z_
		case 1://直线切削给进   做成一样的功能
            rescnt = sscanf(cylib_gcoder.buffer,"G00 X%f Y%f Z%f",&point[0],&point[1],&point[2]);
            if (rescnt == 3){
                //解码成功了
				printf("G00 OK X=%f Y=%f Z=%f\r\n",point[0],point[1],point[2]);
				float x,y,z;
				x = point[0];
				y = point[1];
				z = point[2];
				if (cylib_gcoder_type == 3 || cylib_gcoder_type == 4){
					cylib_controller_move(x,y,z,100);
					//四轴 六轴
					HAL_Delay(10*1000);
				}//cylib_step_motor_run_point_runin(point,100);
				else if (cylib_gcoder_type == 2){
					cylib_b_move(x,y,z);
				}else if (cylib_gcoder_type == 1){
					cylib_delta_move(x,y,z);
				}
            }
            break;
        case 4://暂停
			rescnt = sscanf(cylib_gcoder.buffer,"G04 P%f",&delay);
			if (rescnt == 1){
				HAL_Delay(delay);
			}
            
            break;
        case 90:
			//不处理
            break;
        case 91:
			//不处理
            break;
        default:
			printf("ERR=NOT_FOUND_CODE\r\n");
            break;
    }
}



void cylib_gcoder_decoder_m(void)
{
    uint8_t code = (cylib_gcoder.buffer[1] - '0') * 10 + (cylib_gcoder.buffer[2] - '0');


    switch (code){
        case 2:
			//不处理
            break;
        case 3:
			//不处理
            break;
        case 4:
			//不处理
            break;
        case 5:
			//不处理
            break;
        default:
            break;
    }

}

bool cylib_gcoder_decoder_p(void)
{
    uint8_t code = (cylib_gcoder.buffer[1] - '0') * 10 + (cylib_gcoder.buffer[2] - '0');
	bool ret = false;

    switch (code){
        case 0:
			printf("P00\r\n");
			cylib_gcoder_type = 1;//并联臂
			ret = true;
            break;
        case 1:
			printf("P01\r\n");
			cylib_gcoder_type = 2;//平行臂
			ret = true;
            break;
        case 2:
			printf("P02\r\n");
			cylib_gcoder_type = 3;//四轴
			ret = true;
            break;
        case 3:
			printf("P03\r\n");
			cylib_gcoder_type = 4;//六轴
			ret = true;
            break;
		case 4:
			printf("P04\r\n");
			NVIC_SystemReset();//复位
			break;
        default:
			printf("ERR=NOT_FOUND_CODE\r\n");
            break;
    }
	
	if (ret == true){
		if (cylib_gcoder.isInited == true){
			//此时已经不再支持这个指令
			printf("ERR=NOT_FOUND_CODE\r\n");
		}else{
			cylib_gcoder.isInited = true;
		}
	}else{
		
	}
	
	return ret;

}


bool cylib_gcoder_decoder(void)
{
	bool ret = false;
    int32_t res = cylib_serial_read(cylib_gcoder_uart,&cylib_gcoder.buffer[cylib_gcoder.index],1);

    if (res <= 0){
        return false;
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
			
			case 'P':
                if (cylib_gcoder_decoder_p() == true){
					ret = true;
				}
                break;
			
			
            default:
                break;
        }




        cylib_gcoder.index = 0;

    }




    if (cylib_gcoder.index >= 127){
        cylib_gcoder.index = 0;
    }



	return ret;

}




bool cylib_gcoder_polling(void)
{

    return cylib_gcoder_decoder();

}