/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOCSub.h"
#include "DataProcessing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_BUFFER_NUM 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t Encoder_buffer[ENCODER_BUFFER_NUM] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern MotorRealTimeInformation_str MRT_Inf;
extern MotorParameter_str MotorParameter;
extern SensorData_str SensorData;
extern uint32_t ADC1_Buffer;
extern uint32_t ADC2_Buffer;
extern PI_str D_PI;
extern PI_str Q_PI;
extern PI_str Spd_PI;
extern ControlCommand_str CtrlCom;
extern uint8_t UART2_Buffer[6];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
    SensorData.Ic = ADC1_Buffer & 0xFFFF;
    SensorData.Ic_Ready = 1;
    SensorData.Udc = ADC1_Buffer >> 16;
    SensorData.Udc_Ready = 1;
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
    SensorData.Ia = ADC2_Buffer & 0xFFFF;
    SensorData.Ia_Ready = 1;
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
    static uint16_t MotorStatus = 0;
    
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  //HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
    TIM1->SR &= ~TIM_SR_UIF;
    
    switch(MotorStatus){
    case 0:
        if(SensorData.Theta_Ready == 1){
            MotorStatus = 1;
        }
        break;
    case 1:
        if(SensorData.Udc_Ready == 1) MotorStatus = 2;
        break;
    case 2:
        if(SensorData.Ic_Ready == 1) MotorStatus = 3;
        break;
    case 3:
        if(SensorData.Ia_Ready == 1) {
            MotorStatus = 4;
            SensorData.Theta_Pre = SensorData.Theta;
        }
        break;
    }
    
    switch(MotorStatus){
    case 4:
        MRT_Inf.Udc = (3.3f * SensorData.Udc)  / (1 << 12) * (470 + 15) / 15;
        MRT_Inf.Ia = (1.65f - ((3.3f * SensorData.Ia) / (1 << 12))) / 50 / 0.006f;
        MRT_Inf.Ic = (((3.3f * SensorData.Ic) / (1 << 12)) - 1.65f) / 50 / 0.006f;
    
        MRT_Inf.ThetaE = GetThetaE(MRT_Inf.Theta, MotorParameter.Np);
        Cordic(MRT_Inf.ThetaE, &MRT_Inf.SinTheta, &MRT_Inf.CosTheta);
        Clarke(MRT_Inf.Ia, MRT_Inf.Ic, &MRT_Inf.Ix, &MRT_Inf.Iy);
        Park(MRT_Inf.Ix, MRT_Inf.Iy, MRT_Inf.SinTheta, MRT_Inf.CosTheta, &MRT_Inf.Id, &MRT_Inf.Iq);
        
        MRT_Inf.Ud = PID_Control(&D_PI, CtrlCom.Id, MRT_Inf.Id);
        MRT_Inf.Uq = PID_Control(&Q_PI, CtrlCom.Iq, MRT_Inf.Iq);
        
        InvPark(MRT_Inf.Ud, MRT_Inf.Uq, MRT_Inf.SinTheta, MRT_Inf.CosTheta, &MRT_Inf.Ux, &MRT_Inf.Uy);
        InvClarke(MRT_Inf.Ux, MRT_Inf.Uy, &MRT_Inf.U1, &MRT_Inf.U2, &MRT_Inf.U3);
        GetSector(MRT_Inf.U1, MRT_Inf.U2, MRT_Inf.U3, &MRT_Inf.Sector);
        GetCCR(&MRT_Inf);

        TIM1->CCR1 = (uint32_t)(MRT_Inf.CCRa * Timer_PERIOD);
        TIM1->CCR2 = (uint32_t)(MRT_Inf.CCRb * Timer_PERIOD);
        TIM1->CCR3 = (uint32_t)(MRT_Inf.CCRc * Timer_PERIOD);
        break;
    }
    
    HAL_UART_Receive_IT(&huart2,Encoder_buffer, ENCODER_BUFFER_NUM);
    
    USART2->TDR = 0x02;
    
    
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    GPIOA->BSRR = (uint32_t)GPIO_PIN_5;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    //TIM2->SR &= ~TIM_SR_UIF;
    GetSpd(SensorData.Theta, &SensorData.Theta_Pre, &MRT_Inf.Spd, CtrlCom.SpdFs);
    
    CtrlCom.Id = 0;
    CtrlCom.Iq = PID_Control(&Spd_PI, CtrlCom.Spd, MRT_Inf.Spd);
    GPIOA->BRR = (uint32_t)GPIO_PIN_5;
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
    
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint8_t Encoder_CRC = Encoder_buffer[0] ^ Encoder_buffer[1] ^ Encoder_buffer[2] ^ Encoder_buffer[3] ^ Encoder_buffer[4];
    
    if((Encoder_CRC == Encoder_buffer[5]) && (Encoder_buffer[0] == 0x02)){
        SensorData.Theta = (~((Encoder_buffer[2] << 0) | (Encoder_buffer[3] << 8) | (Encoder_buffer[4] << 16))) & 0x1FFFF;
        MRT_Inf.Theta = (PI * 2 * SensorData.Theta) / (1 << 17);
        SensorData.Theta_Ready = 1;
    }
}
/* USER CODE END 1 */
