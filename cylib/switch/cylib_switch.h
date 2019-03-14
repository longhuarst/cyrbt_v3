



#ifndef CYRBT_V3_CYLIB_SWITCH_H
#define CYRBT_V3_CYLIB_SWITCH_H


#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"


typedef struct{
	struct{
		struct{
			GPIO_TypeDef *port;
			uint32_t pin;
		}pin;
	}instance[4];
}cylib_switch_def;


extern void cylib_switch_init(void);

extern bool cylib_switch_get_status(int x);



#endif //CYRBT_V3_CYLIB_SWITCH_H
