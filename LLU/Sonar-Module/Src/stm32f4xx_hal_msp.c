/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Date               : 28/04/2014 15:53:32
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
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();
  
  /**TIM2 GPIO Configuration  
  PA5   ------> TIM2_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  else if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();
  
  /**TIM3 GPIO Configuration  
  PC6   ------> TIM3_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM9)
  {
    /* Peripheral clock enable */
    __TIM9_CLK_ENABLE();
  
  /**TIM9 GPIO Configuration  
  PE5   ------> TIM9_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  }
  else if(htim_base->Instance==TIM10)
  {
    /* Peripheral clock enable */
    __TIM10_CLK_ENABLE();
  
  /**TIM10 GPIO Configuration  
  PB8   ------> TIM10_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
  else if(htim_base->Instance==TIM11)
  {
    /* Peripheral clock enable */
    __TIM11_CLK_ENABLE();
  
  /**TIM11 GPIO Configuration  
  PB9   ------> TIM11_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
  else if(htim_base->Instance==TIM12)
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
  else if(htim_base->Instance==TIM14)
  {
    /* Peripheral clock enable */
    __TIM14_CLK_ENABLE();
  
  /**TIM14 GPIO Configuration  
  PA7   ------> TIM14_CH1 
  */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();
  
  /**TIM2 GPIO Configuration  
  PA5   ------> TIM2_CH1 
  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
  else if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __TIM3_CLK_DISABLE();
  
  /**TIM3 GPIO Configuration  
  PC6   ------> TIM3_CH1 
  */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM9)
  {
    /* Peripheral clock disable */
    __TIM9_CLK_DISABLE();
  
  /**TIM9 GPIO Configuration  
  PE5   ------> TIM9_CH1 
  */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  }
  else if(htim_base->Instance==TIM10)
  {
    /* Peripheral clock disable */
    __TIM10_CLK_DISABLE();
  
  /**TIM10 GPIO Configuration  
  PB8   ------> TIM10_CH1 
  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

  }
  else if(htim_base->Instance==TIM11)
  {
    /* Peripheral clock disable */
    __TIM11_CLK_DISABLE();
  
  /**TIM11 GPIO Configuration  
  PB9   ------> TIM11_CH1 
  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  }
  else if(htim_base->Instance==TIM12)
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
  else if(htim_base->Instance==TIM14)
  {
    /* Peripheral clock disable */
    __TIM14_CLK_DISABLE();
  
  /**TIM14 GPIO Configuration  
  PA7   ------> TIM14_CH1 
  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

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
