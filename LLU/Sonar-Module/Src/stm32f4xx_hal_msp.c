/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Date               : 24/04/2014 15:35:30
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM12)
  {
    /* Peripheral clock enable */
    __TIM12_CLK_ENABLE();
  
  /**TIM12 GPIO Configuration  
  PB14   ------> TIM12_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  }
  else if(htim_base->Instance==TIM13)
  {
    /* Peripheral clock enable */
    __TIM13_CLK_ENABLE();
  
  /**TIM13 GPIO Configuration  
  PA6   ------> TIM13_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM12)
  {
    /* Peripheral clock disable */
    __TIM12_CLK_DISABLE();
  
  /**TIM12 GPIO Configuration  
  PB14   ------> TIM12_CH1 
  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
  }
  else if(htim_base->Instance==TIM13)
  {
    /* Peripheral clock disable */
    __TIM13_CLK_DISABLE();
  
  /**TIM13 GPIO Configuration  
  PA6   ------> TIM13_CH1 
  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
