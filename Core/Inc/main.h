/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
    float Kp;
    float Ki;
    float Max;
    float up;
    float ui;
    float Error;
    float Out_temp;
}PI_str;

typedef struct{    
    float Spd;

    float Id;
    float Iq;

    float CurTs;
    float SpdTs;
    float CurFs;
    float SpdFs;

    float Ud;
    float Uq;

    uint8_t Mode;
    
    float wc_Current;
    float wc_Speed;
}ControlCommand_str;

typedef struct{
    float Ls;
    float Rs;
    float Kt;
    float J;
    float Flux;
    uint8_t Np;
}MotorParameter_str;

typedef struct{
    float Theta;
    float Spd;
    float Theta_Pre;

    float Udc;

    float SinTheta;
    float CosTheta; 

    float Ux;       
    float Uy;

    float U1;    
    float U2;   
    float U3;       

    uint8_t Sector;

    float CCRa;
    float CCRb; 
    float CCRc;

    float Ia;
    float Ib;
    float Ic;

    float Ix;       
    float Iy;

    float Id;
    float Iq;

    float Ud;
    float Uq;

    float EMF;

    float Ud_qCoupling;
    float Uq_dCoupling;

    float Ud_Electrical;
    float Uq_Electrical;

    float Ud_ElectricalMaxUp;
    float Ud_ElectricalMaxDown;
    float Uq_ElectricalMaxUp;
    float Uq_ElectricalMaxDown;

    float Ex;
    float Ey;

    float ThetaE;
    
    float Tx;
    float Ty;
    
    float Ta;
    float Tb; 
    float Tc;
    
    float Uac;
}MotorRealTimeInformation_str;

typedef struct{
    float Te;
    float TL;
    float Acc;
    float Spd;
    float Spd_Temp;
    float Spd_Bef;
    float Spd_Pre;
    float Theta;
    float Theta_Pre;
    PI_str Spd_PI;
}MotorObserver_str;

typedef struct{
    float Ix_Bef;
    float Iy_Bef;
    float Ex;
    float Ey;
    float Ix;
    float Iy;
    float Vx;
    float Vy;
    float h1;
    float h2;
    float de;
    PI_str SpdE_PI;
    float SpdE;
    float ThetaE;
    float SinTheta;
    float CosTheta;
    float Flag;
    float E1;
    float E2;
    float EMF_LPF_wc;
    float Theta_PLL_wn;
    float Theta_PLL_we;
    float Theta_PLL_zeta;
    float Spd_LPF_wc;
    float Switch_Spd;
    float ThetaE2;
    float EMF_Flag;
}SlidingModeObserver_str;

typedef struct{
    uint32_t Theta;
    uint32_t ThetaE;
    uint32_t Theta_Pre;
    int16_t Ia;
    int16_t Ic;
    uint16_t Udc;
    uint8_t Udc_Ready;
    uint8_t Ia_Ready;
    uint8_t Ic_Ready;
    uint8_t Theta_Ready;
    uint8_t ADC1_DMA_Ready;
    uint8_t ADC2_DMA_Ready;
    uint8_t Encoder_Ready;
}SensorData_str;

typedef union{
	uint32_t DMAData;
	uint16_t SensorData[2];
}ADCData_union;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SDabc_Pin GPIO_PIN_15
#define SDabc_GPIO_Port GPIOB
#define INa_Pin GPIO_PIN_8
#define INa_GPIO_Port GPIOA
#define INb_Pin GPIO_PIN_9
#define INb_GPIO_Port GPIOA
#define INc_Pin GPIO_PIN_10
#define INc_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define TIM_Fre (uint32_t)(170000000)
#define PWM_Fre (uint32_t)(20000)
#define Timer_PERIOD (uint32_t)(TIM_Fre / PWM_Fre / 2)
#define PWM_phA_Init (uint32_t)(Timer_PERIOD / 2)
#define PWM_phB_Init (uint32_t)(Timer_PERIOD / 2)
#define PWM_phC_Init (uint32_t)(Timer_PERIOD / 2)

#define TRUE 1
#define FALSE 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
