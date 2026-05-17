// /* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file    tim.c
//   * @brief   This file provides code for the configuration
//   *          of the TIM instances.
//   ******************************************************************************
//   * @attention
//   *
//   * Copyright (c) 2026 STMicroelectronics.
//   * All rights reserved.
//   *
//   * This software is licensed under terms that can be found in the LICENSE file
//   * in the root directory of this software component.
//   * If no LICENSE file comes with this software, it is provided AS-IS.
//   *
//   ******************************************************************************
//   */
// /* USER CODE END Header */
// /* Includes ------------------------------------------------------------------*/
// #include "tim.h"
//
// /* USER CODE BEGIN 0 */
// DMA_HandleTypeDef hdma_tim1_up;
// /* USER CODE END 0 */
//
// TIM_HandleTypeDef htim1;
// TIM_HandleTypeDef htim3;
//
// /* TIM1 init function */
// void MX_TIM1_Init(void)
// {
//     TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//     TIM_MasterConfigTypeDef sMasterConfig = {0};
//     TIM_OC_InitTypeDef sConfigOC = {0};
//     TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//     htim1.Instance = TIM1;
//     htim1.Init.Prescaler = 0;
//     htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim1.Init.Period = 344 - 1; // 周期500，800kHz
//     htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim1.Init.RepetitionCounter = 0;
//     htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//
//     if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
//         Error_Handler();
//     }
//
//     sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//     if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
//         Error_Handler();
//     }
//
//     if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
//         Error_Handler();
//     }
//
//     sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
//     sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//     sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//     if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
//         Error_Handler();
//     }
//
//     sConfigOC.OCMode = TIM_OCMODE_PWM1;
//     sConfigOC.Pulse = 0;
//     sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//     sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//     sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//     sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//     sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//
//     if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
//         Error_Handler();
//     }
//
//     sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
//     sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
//     sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//     sBreakDeadTimeConfig.DeadTime = 0;
//     sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//     sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//     sBreakDeadTimeConfig.BreakFilter = 0;
//     sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//     sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//     sBreakDeadTimeConfig.Break2Filter = 0;
//     sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
//
//     if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }
// }
//
// /* TIM3 init function */
// void MX_TIM3_Init(void)
// {
//     TIM_MasterConfigTypeDef sMasterConfig = {0};
//     TIM_OC_InitTypeDef sConfigOC = {0};
//
//     htim3.Instance = TIM3;
//     htim3.Init.Prescaler = 24-1;
//     htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim3.Init.Period = 65535;
//     htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//     if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//     sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//     if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     sConfigOC.OCMode = TIM_OCMODE_PWM1;
//     sConfigOC.Pulse = 0;
//     sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//     sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//     if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     HAL_TIM_MspPostInit(&htim3);
// }
//
// void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
// {
//     if(tim_baseHandle->Instance == TIM1)
//     {
//         __HAL_RCC_TIM1_CLK_ENABLE();
//         __HAL_RCC_DMA2_CLK_ENABLE();
//
//         HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 5, 0);
//         HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
//         HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
//         HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
//         HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 5, 0);
//         HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
//         HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
//         HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
//
//         HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
//         HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
//     }
// }
//
// void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
// {
//     if(tim_pwmHandle->Instance == TIM3)
//     {
//         __HAL_RCC_TIM3_CLK_ENABLE();
//     }
// }
//
// void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//     if(timHandle->Instance == TIM1)
//     {
//         __HAL_RCC_GPIOE_CLK_ENABLE();
//
//         GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_13;
//         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//         GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//         HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//         hdma_tim1_up.Instance = DMA2_Stream1;
//         hdma_tim1_up.Init.Request = DMA_REQUEST_TIM1_UP;
//         hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
//         hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
//         hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
//         hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//         hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//         hdma_tim1_up.Init.Mode = DMA_NORMAL;
//         hdma_tim1_up.Init.Priority = DMA_PRIORITY_HIGH;
//         HAL_DMA_Init(&hdma_tim1_up);
//
//         __HAL_LINKDMA(timHandle, hdma[TIM_DMA_ID_UPDATE], hdma_tim1_up);
//     }
//
//     if(timHandle->Instance == TIM3)
//     {
//         __HAL_RCC_GPIOB_CLK_ENABLE();
//
//         GPIO_InitStruct.Pin = GPIO_PIN_1;
//         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//         GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//         HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//     }
// }
//
// void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
// {
//     if(tim_baseHandle->Instance == TIM1)
//     {
//         __HAL_RCC_TIM1_CLK_DISABLE();
//
//         HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
//         HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn);
//         HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
//         HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
//         HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
//     }
// }
//
// void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
// {
//     if(tim_pwmHandle->Instance == TIM3)
//     {
//         __HAL_RCC_TIM3_CLK_DISABLE();
//     }
// }
//
// /* USER CODE BEGIN 1 */
//
// /* USER CODE END 1 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
DMA_HandleTypeDef hdma_tim1_up;
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 10000;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 2750;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // 配置TIM1_CH1
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    // 配置TIM1_CH3
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 10000;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2750;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // 配置TIM2_CH1
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    // 配置TIM2_CH3
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 10000;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 2750;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim3);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
        HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

        HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    if(tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if(tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(timHandle->Instance == TIM1)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();

        // TIM1_CH1: PE9, TIM1_CH3: PE13
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        hdma_tim1_up.Instance = DMA2_Stream1;
        hdma_tim1_up.Init.Request = DMA_REQUEST_TIM1_UP;
        hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_tim1_up.Init.Mode = DMA_NORMAL;
        hdma_tim1_up.Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&hdma_tim1_up);

        __HAL_LINKDMA(timHandle, hdma[TIM_DMA_ID_UPDATE], hdma_tim1_up);
    }
    else if(timHandle->Instance == TIM2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // TIM2_CH1: PA0, TIM2_CH3: PA2
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timHandle->Instance == TIM3)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // TIM3_CH4: PB1
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if(tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();

        HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    if(tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
    else if(tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */