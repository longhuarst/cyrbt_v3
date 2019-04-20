/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "../../cylib/cylib.h"
//#include "stm32f1xx_hal_rcc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint64_t user_timestamp(void)
{
	return ((uint64_t)TIM5->CNT << 48) |
		((uint64_t)TIM4->CNT << 32) |
		((uint64_t)TIM3->CNT << 16) |
		((uint64_t)TIM2->CNT << 0 ) ;
}

void user_delay_us(uint32_t delay)
{
	uint64_t ts1 = user_timestamp();
	uint64_t ts2;
	
	
	do{
		ts2 = user_timestamp();
		
		if (ts2 - ts1 >= delay){
			break;
		}
	}while(1);
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim5);


    cylib_init();//鍒濆鍖朇YLIB搴?
	
	if (!__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)){
		printf("wait for cmd \r\n");
	
		while(!cylib_gcoder_polling());
	}
	
	
	
	//选择机械臂类型
	
	
	
	
	//////////////////////////////////////////平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂
	{
		
		//j28接中间电机的限位开关   CH3
		//j18接中心电机[2]
		//j29接上下电机的限位开关   CH4
		//j17接中间电机[1]
		//j16接上下电机[0]
		
		//电机校准 【水平】
		if (cylib_switch_get_status(3) == false){
			//旋转180*1.2度
			cylib_step_motor_run_async_motor2(100,-128000*1.2);
			
			while(cylib_switch_get_status(3) == false);
		
			NVIC_SystemReset();		
		}
		
		
		//校准【中间】电机
		
		cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_SET);
		
		while(1){
			
			if (HAL_GPIO_ReadPin(Limit_SW_CH3_GPIO_Port,Limit_SW_CH3_Pin)){
				break;
			}
			
			
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_SET);
			user_delay_us(100);
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_RESET);
			user_delay_us(100);
		}
		
		
		while(1){
			
			if (HAL_GPIO_ReadPin(Limit_SW_CH4_GPIO_Port,Limit_SW_CH4_Pin)){
				break;
			}
			
			
			cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_SET);
			user_delay_us(100);
			cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_RESET);
			user_delay_us(100);
		}
		
		
		
		
		///////////////////////////////////到达校准位置到达校准位置到达校准位置到达校准位置
		
		
		cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.dir,GPIO_PIN_RESET);
//6400 90都
		for (int i=0; i< 72000; ++i){
			
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_SET);
			user_delay_us(100);
			cylib_step_motor_io_write(cylib_step_motor.instance[1].pin.clk,GPIO_PIN_RESET);
			user_delay_us(100);
		}
		
		
		
		
	}
	//////////////////////////////////////////平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂平行臂
	
	
	
	
	
	
	while(1);
	
	
	
	
	//////////////////////////////////////////四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴
	{
	
		
		
		while (cylib_angle_wait_init() != 0x7u){
			
			printf("wait angle sensors inited !\r\n");
			HAL_Delay(1000);
			
			
		}
		printf("all angle sensors has been inited !\r\n");

	//	cylib_step_motor_run_async_motor0(20,10000);
	//	cylib_step_motor_run_async_motor1(20,10000);
		
		//电机校准 【水平】
		if (cylib_switch_get_status(3) == false){
			//旋转180*1.2度
			cylib_step_motor_run_async_motor2(100,-128000*1.2);
			
			while(cylib_switch_get_status(3) == false);
		
			NVIC_SystemReset();		
		}
		
		HAL_Delay(12000);//等待12秒 惯导稳定
		
		//先校准内电机
		for (int i=0;i<10;++i)
		{
			//读取惯导0数据
			float angle = 0.0f;
			while (cylib_angle_get(0,&angle) == false);
			printf("motor1 angle =  %f\r\n",angle);
			
			//电机2 角度为 45度
			int32_t step = (45.0f - angle ) / 0.00046875f;
			
			printf("error degree = %f , step = %d\r\n",45-angle, step);
			
			cylib_step_motor_run_async_motor1(100,step);
			
			cylib_step_motor_block_wait_for_all();//等待电机完成
			
			printf("motor1 complete \r\n");
		}
		
		printf("-------------------- \r\n");
		
		//电机校准外电机
		for (int i=0;i<10;++i)
		{
			//读取惯导0数据
			float angle = 0.0f;
			while (cylib_angle_get(1,&angle) == false);
			printf("motor0 angle =  %f\r\n",angle);
			
			//电机2 角度为 45度
			int32_t step = -(-45.0f - angle ) / 0.00046875f;
			
			printf("error degree = %f , step = %d\r\n",-45-angle, step);
			
			cylib_step_motor_run_async_motor0(100,step);
			
			cylib_step_motor_block_wait_for_all();//等待电机完成
			
			printf("motor0 complete \r\n");
		}
		
		printf("-----------------\r\n");
		
		for (int i=0;i<10;++i){
			HAL_Delay(1000);
			printf("angle 0 = %f\r\n",cylib_angle[0].y);
			printf("angle 1 = %f\r\n",cylib_angle[1].y);
			printf("angle 2 = %f\r\n",cylib_angle[2].y);

		}
		
	}
		
	
	//////////////////////////////////////////四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴四轴
	
	
	
	
	
	
	//清除坐标
	cylib_location_set(0,0,0,0);
	
	
	//-------------------------校准完毕----------------
	
	
	#if 0
	cylib_controller_move(0,150,150,100);
	
	
	HAL_Delay(10*1000);
	
	cylib_controller_move(0,140,100,100);
	
	HAL_Delay(10*1000);
	
	cylib_controller_move(-50,140,100,100);
	
	
	
//	while(1){
//		HAL_Delay(1000);
//		
//		if (cylib_switch_get_status(0)){
//			printf("sw0 on\r\n");
//		}else{
//			printf("sw0 off\r\n");
//		}
//		
//		if (cylib_switch_get_status(1)){
//			printf("sw1 on\r\n");
//		}else{
//			printf("sw1 off\r\n");
//		}
//		
//		if (cylib_switch_get_status(2)){
//			printf("sw2 on\r\n");
//		}else{
//			printf("sw2 off\r\n");
//		}
//		
//		if (cylib_switch_get_status(3)){
//			printf("sw3 on\r\n");
//		}else{
//			printf("sw3 off\r\n");
//		}
//		
//	}
//	
	
	#endif
	
	
	printf(" \
------------------------------\r\n \
system run into polling ..... \r\n \
------------------------------\r\n");
	
//	cylib_step_motor_run_async_motor0(20,10000);
//	cylib_step_motor_run_async_motor1(20,10000);
//	cylib_step_motor_run_async_motor1(200,10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



      cylib_gcoder_polling();//G浠ｇ爜瑙ｆ瀽
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM1){
    cylib_step_motor_timer_callback0();//鐢ㄤ簬鐢垫満0鐨勫畾鏃跺櫒
  }else if (htim->Instance == TIM2){

  }else if (htim->Instance == TIM3){

  }else if (htim->Instance == TIM4){

  }else if (htim->Instance == TIM5){

  }else if (htim->Instance == TIM6){

  }else if (htim->Instance == TIM7){
    cylib_step_motor_timer_callback1();//鐢ㄤ簬鐢垫満1鐨勫畾鏃跺櫒
  }else if (htim->Instance == TIM8){
    cylib_step_motor_timer_callback2();//鐢ㄤ簬鐢垫満2鐨勫畾鏃跺櫒
  }



}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1){
    cylib_serial_rx_callback(0);
  }else if (huart->Instance == USART2){
    cylib_serial_rx_callback(1);
  }else if (huart->Instance == USART3){
    cylib_serial_rx_callback(2);
  }

}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
