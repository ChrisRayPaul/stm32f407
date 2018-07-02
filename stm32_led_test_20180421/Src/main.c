
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "../user_code/devices/Inc/dev_out_pwm.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 0xffff);
	return len;
}
uint8_t ch;
uint8_t rxbuf[10];
uint8_t rx_c=0;
uint32_t count = 0;
#define DATA_SIZE	(4U)

/* pwm duty */
uint32_t led_pwm_duty = 500;
uint32_t led_pwm_pulse = 0;

typedef struct tag_UartFrame {
	uint8_t start_frame;
	uint8_t verify_frame;
	uint8_t stop_frame;
	uint8_t length;
	uint8_t count;
	uint8_t data[DATA_SIZE];

}uart_frame;

/* uart recv frame */
uart_frame recv_frame =
{
		.start_frame = 0xAA,
		.verify_frame = 0x00,
		.stop_frame = 0xFF,
		.length = 4,
		.count = 0,
		.data = {0}
};

enum FrameState{fs_init, fs_start, fs_verify, fs_stop};

enum FrameState frame_state = fs_init;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	if (UartHandle == &huart1) {

		//HAL_UART_Receive_IT(&huart1,&ch,1);
		/* state machine */
		switch(frame_state)
		{
		case fs_init:
			if(recv_frame.start_frame == ch)
			{
				frame_state = fs_start;
				printf("now init goto start\n");
			}
			break;
		case fs_start:
			recv_frame.data[recv_frame.count] = ch;
			recv_frame.count++;
			printf("now recv data[%d]\n", recv_frame.count);
			if(recv_frame.count >= recv_frame.length)
			{
				frame_state = fs_verify;
				recv_frame.count = 0;
				printf("start goto verify\n");
			}
			break;
		case fs_verify:
			recv_frame.verify_frame = ch;
			frame_state = fs_stop;
			printf("now verify goto stop\n");
			break;
		case fs_stop:
			if(recv_frame.stop_frame == ch)
			{
				//led_pwm_duty = 0;
				frame_state = fs_init;
				printf("now stop goto init\n");
				uint32_t data_sum = 0;
				uint8_t verify_sum = 0;
				for(uint8_t i = 0; i < DATA_SIZE; i++)
				{
					data_sum += (uint32_t)recv_frame.data[i];
				}
				verify_sum = (uint8_t)(recv_frame.start_frame +
						     recv_frame.stop_frame + data_sum);
				printf("verity_sum: %d data_sum: %d verify:%d\n", verify_sum, data_sum, recv_frame.verify_frame);
				if(verify_sum == recv_frame.verify_frame)
				{
					led_pwm_duty = ((uint32_t)recv_frame.data[0])<<24 |
								   ((uint32_t)recv_frame.data[1])<<16 |
								   ((uint32_t)recv_frame.data[2])<<8 |
								   ((uint32_t)recv_frame.data[3]);

					printf("led duty:%d\n", led_pwm_duty);
					for(uint8_t i = 0; i < 4; i++)
					{
						printf("data[%d]: %d\n",i,recv_frame.data[i]);
					}
				}
			}
			break;
		default:
			break;
		}


		rxbuf[rx_c++]=ch;
		if(ch != 0xAA)
		{
			puts(".");
		}
		else
		{
			count++;
			printf("I'm AA %d\n", count);
		}
		if(rx_c>=10)
		{
			rx_c=0;
		}
		HAL_UART_Receive_IT(&huart1,&ch,1);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_UART_MspInit(&huart1);
  if (HAL_UART_Receive_IT(&huart1, &ch, 1) != HAL_OK)
  		Error_Handler();

  uint32_t pwm_pulse = 0;
  uint32_t step = 5;
#if 0
  dev_out_pwm_set_period(20);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
#if 0
		printf("hello\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(500);

		dev_out_pwm_set_pulse(pwm_pulse);
		pwm_pulse += step;
		if((1000 == pwm_pulse) || (0 == pwm_pulse))
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			step = -step;
		}
#endif
		dev_out_pwm_set_pulse(led_pwm_duty);
		HAL_Delay(5);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
