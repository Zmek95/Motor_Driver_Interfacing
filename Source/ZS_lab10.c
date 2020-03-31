/* ZS_lab10.c: PWM controls for timer
 *	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *  Date: Mar 11, 2020
 */
#include <stdio.h>
#include <stdint.h>
#include "common.h"

TIM_HandleTypeDef tim3;
//TIM_HandleTypeDef tim4;

// FUNCTION      : encoderInit()
// DESCRIPTION   : Intializes the GPIO pins, timers and encoder to be configured for quadrautre encoder output 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void encoderInit(void) {
	
	HAL_StatusTypeDef rc;
	GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
	TIM_Encoder_InitTypeDef encoderConfig;
	
	// Enabling clock for GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//configure GPIO pins	
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//Enabling clock for TIM3
	__HAL_RCC_TIM3_CLK_ENABLE();
	//Configuring Timer 3
	tim3.Instance = TIM3;
	tim3.Init.Prescaler = 0;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3.Init.Period = 0xffff;
	tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim3.Init.RepetitionCounter = 0;
	rc = HAL_TIM_Base_Init(&tim3);
	if(rc != HAL_OK) {
		printf("Failed to initialize Timer 3 Base, rc=%u\n",rc);
		return;
	}
	/*//Enabling clock for TIM4
	__HAL_RCC_TIM4_CLK_ENABLE();
	//Configuring Timer 4
	tim4.Instance = TIM4;
	tim4.Init.Prescaler = 0;
	tim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim4.Init.Period = 0xffff;
	tim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim4.Init.RepetitionCounter = 0;
	rc = HAL_TIM_Base_Init(&tim4);
	if(rc != HAL_OK) {
		printf("Failed to initialize Timer 3 Base, rc=%u\n",rc);
		return;
	}*/
	
	//Configuring encoder
	encoderConfig.EncoderMode = TIM_ENCODER_TI12;
	encoderConfig.IC1Polarity = 0;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC1Prescaler = 0;
	encoderConfig.IC1Filter = 3;
	encoderConfig.IC2Polarity = 0;
	encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC2Prescaler = 0;
	encoderConfig.IC2Filter = 3;
	//Initialize encoder
	rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);
	if(rc != HAL_OK) {
		printf("Failed to initialize Timer 3 Encoder, rc=%u\n",rc);
		return;
	}
	/*rc = HAL_TIM_Encoder_Init(&tim4, &encoderConfig);
	if(rc != HAL_OK) {
		printf("Failed to initialize Timer 4 Encoder, rc=%u\n",rc);
		return;
	}*/
	//Start Encoder
	rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder, rc=%u\n",rc);
		return;
	}
	rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder, rc=%u\n",rc);
		return;
	}
	/*rc = HAL_TIM_Encoder_Start(&tim4, TIM_CHANNEL_1);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 4 Encoder, rc=%u\n",rc);
		return;
	}
	rc = HAL_TIM_Encoder_Start(&tim4, TIM_CHANNEL_2);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 4 Encoder, rc=%u\n",rc);
		return;
	}*/
	
	//Enable interrupts, use Capture compare for generating interrupt
	HAL_NVIC_SetPriority(TIM3_IRQn,2,3);
	NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_SetPriority(TIM4_IRQn,3,4);
	NVIC_EnableIRQ(TIM4_IRQn);
}

uint16_t readEncoder(void){
	uint16_t encoderValue;
	encoderValue = TIM3->CNT;
	return encoderValue;
}







