/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 24/04/2014 15:35:30
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin( GPIOD, GPIO_PIN_12, GPIO_PIN_SET );
  HAL_GPIO_WritePin( GPIOD, GPIO_PIN_14, GPIO_PIN_SET );
  HAL_TIM_IC_Start_IT( &htim12, TIM_CHANNEL_2 );
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
	  HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_12 );
	  HAL_TIM_PWM_Start( &htim13, TIM_CHANNEL_1 );
	  
	  HAL_Delay( 100 );	  
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* TIM12 init function */
void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_SlaveConfigTypeDef sSlaveConfig;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim12);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim12);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_2);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim12, &sSlaveConfig);

}

/* TIM13 init function */
void MX_TIM13_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim13);

  HAL_TIM_PWM_Init(&htim13);

  HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE);

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 65526;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
uint32_t ic_val;
uint32_t range_mm; 
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	ic_val = HAL_TIM_ReadCapturedValue( &htim12, TIM_CHANNEL_2 );	
	range_mm = (int)((float)ic_val*0.175f+0.5f);
	HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_14 );
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
