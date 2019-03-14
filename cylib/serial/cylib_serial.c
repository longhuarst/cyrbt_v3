//
// Created by Rebecca on 2019/2/17.
//

//#include <stm32f1xx_hal_conf.h>
#include "cylib_serial.h"

#include "../buffer/cylib_ringbuffer.h"

#include "usart.h"


typedef struct{
    struct{
        struct{
            ring_buffer_t rb;
            uint8_t dma_temp;
        }tx;
        struct{
            ring_buffer_t rb;
            uint8_t dma_temp;
        }rx;
        UART_HandleTypeDef *huart;
    }instance[3];
}cylib_serial_def;



cylib_serial_def cylib_serial = {
        .instance = {
                {
                        .tx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .rx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .huart = &huart1,//RS232
                },{
                        .tx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .rx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .huart = &huart2,//SPP-CA
                },{
                        .tx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .rx = {
                                .dma_temp = 0,
                                .rb = {
                                        .buffer = {0,},
                                        .head_index = 0,
                                        .tail_index = 0,
                                }
                        },
                        .huart = &huart3,//RS485
                }
        }

};

void cylib_serial_init(void)
{


    HAL_UART_Receive_DMA(&huart1,&cylib_serial.instance[0].rx.dma_temp,1);
    HAL_UART_Receive_DMA(&huart2,&cylib_serial.instance[1].rx.dma_temp,1);
    HAL_UART_Receive_DMA(&huart3,&cylib_serial.instance[2].rx.dma_temp,1);
}



void cylib_serial_rx_callback(int x)
{
    if (x >=0 && x < 3){

        ring_buffer_queue(&cylib_serial.instance[x].rx.rb,cylib_serial.instance[x].rx.dma_temp);


    }

}



int32_t cylib_serial_read(int32_t fd, uint8_t *buffer, int32_t length)
{
    if (length <= 0 || buffer == 0){
        return -1;
    }

    if (fd >= 0 && fd < 3){

        int32_t tlen = ring_buffer_num_items(&cylib_serial.instance[fd].rx.rb);

        if (tlen < length)
            length = tlen;

        ring_buffer_dequeue_arr(&cylib_serial.instance[fd].rx.rb,(char *)buffer,length);

        return length;

    }

    return -1;
}




















int fputc(int ch, FILE *f){
	//里面是要重定向的设备显示/发送一个字节的代码
	while(HAL_UART_Transmit(&huart1,(char *)&ch,1,1000)!=HAL_OK);
};

	
	
	
	
