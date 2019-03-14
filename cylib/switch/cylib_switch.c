#include "cylib_switch.h"

#include <stdbool.h>
#include <string.h>



cylib_switch_def cylib_switch = {
	.instance = {
		{
			.pin = {
				.port = Proximity_SW_CH1_GPIO_Port,
				.pin = Proximity_SW_CH1_Pin,
			}
		},{
			.pin = {
				.port = Proximity_SW_CH2_GPIO_Port,
				.pin = Proximity_SW_CH2_Pin,
			}
		},{
			.pin = {
				.port = Proximity_SW_CH3_GPIO_Port,
				.pin = Proximity_SW_CH3_Pin,
			}
		},{
			.pin = {
				.port = Proximity_SW_CH4_GPIO_Port,
				.pin = Proximity_SW_CH4_Pin,
			}
		},
	}
};

void cylib_switch_init(void)
{
	
}



bool cylib_switch_get_status(int x)
{
	if (x < 0 || x >= 4){
		return false;
	}
	
	return HAL_GPIO_ReadPin(cylib_switch.instance[x].pin.port,cylib_switch.instance[x].pin.pin);
}
