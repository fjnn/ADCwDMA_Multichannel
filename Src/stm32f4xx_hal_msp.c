/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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

extern DMA_HandleTypeDef hdma_adc2;

extern DMA_HandleTypeDef hdma_adc3;

extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO Configuration    
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    PB0     ------> ADC2_IN8
    PB1     ------> ADC2_IN9 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream3;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    /**ADC3 GPIO Configuration    
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA2_Stream1;
    hdma_adc3.Init.Channel = DMA_CHANNEL_2;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc3);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
    PB0     ------> ADC2_IN8
    PB1     ------> ADC2_IN9 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB11     ------> USART3_RX
    PD8     ------> USART3_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB11     ------> USART3_RX
    PD8     ------> USART3_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
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
