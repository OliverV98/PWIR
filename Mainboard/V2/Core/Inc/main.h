/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define MOTORS_MIN_CCR 20000
#define MOTORS_MAX_CCR 65000
#define MOTORS_MAX_SPEED 100
#define MOTORS_CO ((MOTORS_MAX_CCR - MOTORS_MIN_CCR) / MOTORS_MAX_SPEED)

#define ESC_IDLE_CCR 4000
#define ESC_MIN_CCR 4200
#define ESC_MAX_CCR 7000
#define ESC_MAX_SPEED 100
#define ESC_CO ((ESC_MAX_CCR - ESC_MIN_CCR) / ESC_MAX_SPEED)

#define SERVO_IN_MIN_CCR 4700
#define SERVO_OUT_MIN_CCR 5000
#define SERVO_MAX_SPEED 100
#define SERVO_CO (400/SERVO_MAX_SPEED)

#define ENCODER_MAX 65535
#define ENCODER_QUADRANT (ENCODER_MAX / 4)
#define ENCODER_QUADRANT_3 (ENCODER_QUADRANT * 3)
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
    TM_RE_Rotate_Increment, /*!< Encoder was incremented */
    TM_RE_Rotate_Decrement, /*!< Encoder was decremented */
    TM_RE_Rotate_Nothing    /*!< Encoder stop at it was before */
} TM_RE_Rotate_t;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum {
    TM_RE_Mode_Zero,        /*!< Rotary encoder mode zero. It is used for direction when it will be increment od decrement, default used */
    TM_RE_Mode_One          /*!< Rotary encoder mode one. It is used for direction when it will be increment od decrement */
} TM_RE_Mode_t;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)
#define clamp(l, h, val) max(l, min(h, val))
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC3A_Pin GPIO_PIN_1
#define ENC3A_GPIO_Port GPIOF
#define ENC2_A_Pin GPIO_PIN_0
#define ENC2_A_GPIO_Port GPIOA
#define ENC1A_Pin GPIO_PIN_2
#define ENC1A_GPIO_Port GPIOA
#define ENC1B_Pin GPIO_PIN_3
#define ENC1B_GPIO_Port GPIOA
#define ENC1B_EXTI_IRQn EXTI3_IRQn
#define ENC3B_Pin GPIO_PIN_4
#define ENC3B_GPIO_Port GPIOA
#define ENC3B_EXTI_IRQn EXTI4_IRQn
#define ESCPWM_Pin GPIO_PIN_5
#define ESCPWM_GPIO_Port GPIOA
#define M1_PWM1_Pin GPIO_PIN_6
#define M1_PWM1_GPIO_Port GPIOA
#define M1_PWM2_Pin GPIO_PIN_7
#define M1_PWM2_GPIO_Port GPIOA
#define M2_PWM1_Pin GPIO_PIN_8
#define M2_PWM1_GPIO_Port GPIOA
#define M2_PWM2_Pin GPIO_PIN_9
#define M2_PWM2_GPIO_Port GPIOA
#define SERVOPWM_Pin GPIO_PIN_15
#define SERVOPWM_GPIO_Port GPIOA
#define M3_PWM1_Pin GPIO_PIN_6
#define M3_PWM1_GPIO_Port GPIOB
#define M3_PWM2_Pin GPIO_PIN_7
#define M3_PWM2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef struct Motor {
	uint16_t prev_pos;
	uint16_t cur_pos;
	int cur_enc_speed;

	int enc_speed_hist[10];
	int enc_speed_hist_cnt;
	float enc_speed_hist_avg;

	float target_speed;
	float cur_speed;
	float prev_speed;
	int err_sum;
} Motor;

typedef struct {
    int32_t Absolute;                /*!< Absolute rotation from beginning, for public use */
    int32_t Diff;                     /*!< Rotary difference from last check, for public use */
    TM_RE_Rotate_t Rotation;        /*!< Increment, Decrement or nothing, for public use */
    TM_RE_Mode_t Mode;              /*!< Rotary encoder mode selected */
    uint8_t LastA;                  /*!< Last status of A pin when checking. Meant for private use */
    int32_t RE_Count;               /*!< Temporary variable to store data between rotation and user check */
    GPIO_TypeDef* GPIO_A;           /*!< Pointer to GPIOx for Rotary encode A pin. Meant for private use */
    GPIO_TypeDef* GPIO_B;           /*!< Pointer to GPIOx for Rotary encode B pin. Meant for private use */
    uint16_t GPIO_PIN_A;            /*!< GPIO pin for rotary encoder A pin. This pin is also set for interrupt */
    uint16_t GPIO_PIN_B;            /*!< GPIO pin for rotary encoder B pin. */
} TM_RE_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
