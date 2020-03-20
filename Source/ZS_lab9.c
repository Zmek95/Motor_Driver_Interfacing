/* 	ZS_lab9.c
*	DC Motor controls
* 	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
*	Date: Mar 19, 2020
*/

#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "DC_CommandQueue.h"
//prototypes
void PWMInit(void);
void DCInit(void);
void motorStop(uint16_t);
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction);

//global variables
uint32_t counter[2]	  = {0,0};
uint8_t  timerDone[2] = {1,1};

void DCInit(void){
		GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
		TIM_OC_InitTypeDef  sConfig;
	// Enabling clock for GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	//configure GPIO pins for Motor1
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//configure GPIO pins for Motor2
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//Making sure motors are not activated
	stopMotor(1);
	stopMotor(2);
	//configuring TIM1
	__HAL_RCC_TIM1_CLK_ENABLE();
	tim1.Instance = TIM1;
	tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
	tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim1.Init.Period = 1000;
	tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim1.Init.RepetitionCounter = 0;
	TIM1 -> CR1 |= (TIM_CR1_URS); //only counter over/under flow generates interrupt
	TIM1 -> DIER |= 0b1; //setting Update Interrupt Enable bit 
	TIM1 -> EGR |= 0b1; //setting Update Generation bit 
	HAL_TIM_Base_Init(&tim1);
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.Pulse = 0;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  //Enable Interrupts 
	HAL_NVIC_SetPriority(TIM1_IRQn,0,1); 
	NVIC_EnableIRQ(TIM1_IRQn);
}
void stopMotor(uint16_t channel){
	if(channel == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	}
}

// FUNCTION      : DC()
// DESCRIPTION   : Sets the duty cycle for the PWM waveform for a specified channel 
// PARAMETERS    : uint16_t channel - 2 channels that can generate PWM waveforms independently
//                 uint16_t dutyCycle - duty cycle for the PWM waveform
//				   uint16_t direction - direction of rotation
// RETURNS       : Nothing
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction){
	dutyCycle = dutyCycle * 10;	//scaling up to 1000 microseconds per overflow
	TIM1->CR1 &= ~TIM_CR1_CEN;	//stopping timer
	if (channel == 1) {
		TIM1->CCR1 = dutyCycle;
		if(direction == 1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
		}
	}
	else{
		TIM1->CCR2 = dutyCycle;
		if(direction == 1){
			if(direction == 1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
		}
	}
	TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
}
/************************Commands*******************************/
ParserReturnVal_t CmdDCInit(int mode) {

	if (mode != CMD_INTERACTIVE) return CmdReturnOk;

	DCInit();
	return CmdReturnOk;
}
ADD_CMD("DCInit", CmdDCInit, "	      Initializes DC motor driver, TIM1 with PWM and interrupt")

ParserReturnVal_t CmdDC(int mode) {

	uint16_t channel;  //used to choose which motor to be activated
	uint16_t dutyCycle;//used to control speed of motor
	uint16_t direction;//used to set direction of rotation
	uint16_t time;	   //time in milliseconds
	uint16_t rc;

	if (mode != CMD_INTERACTIVE) return CmdReturnOk;

	rc = fetch_uint16_arg(&channel);	//inputting channel number from user
	if (rc) {
		printf("Please select which motor to control\n");
		return CmdReturnBadParameter1;
	}
	if (channel > 2 || channel < 1) {	//checking if channel number is valid 
		printf("Type either 1 or 2 for motor selection\n");
		return CmdReturnBadParameter1;
	}
	rc = fetch_uint16_arg(&direction);	//inputting direction number from user
	if (rc) {
		printf("Please select direction of rotation\n");
		return CmdReturnBadParameter3;
	}
	if (direction > 2 || direction < 0) {	//checking if direction is valid 
		printf("Type 1 for CW, 2 for CCW, 0 or 3 to stop motor\n");
		return CmdReturnBadParameter3;
	}
	if(direction == 1 || direction == 2){
		rc = fetch_uint16_arg(&time);	//inputting direction number from user
		if (rc) {
			printf("Please enter time in milliseconds\n");
			return CmdReturnBadParameter4;
		}
		if (time < 0 || time > 30000) {	//checking if time is valid 
			printf("time can't be negative or greater than 30000 ms\n");
			return CmdReturnBadParameter4;
		}
		rc = fetch_uint16_arg(&dutyCycle);	//inputting speed from user
		if (rc) {
			printf("Please enter the speed, (0 to 100)\n");
			return CmdReturnBadParameter2;
		}
		if (dutyCycle > 100 || dutyCycle < 0) {	//checking if speed selected is valid 
			printf("speed must be between 0 and 100\n");
			return CmdReturnBadParameter2;
		}
	}
	//DC(channel, dutyCycle, direction, time);
	addToQueue(channel, dutyCycle, direction, time);
	return CmdReturnOk;
}
ADD_CMD("DC", CmdDC, "<channel><direction><speed><time>  activates selected motor and sets direction, speed and operating time if stop is requested")
/***************ISR***************/
// FUNCTION      : TIM1_CC_IRQHandler
// DESCRIPTION   : 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_CC_IRQHandler(void) {

	TIM1->CR1 &= ~TIM_CR1_CEN; 	//stopping timer
	if(counter[0] == 0){//checking if channel 1 timed-out
		motorStop(1);	//stopping motor 1
	}
	else{
		counter[0]--;
	}	
	if(counter[1] == 0){//checking if channel 2 timed-out
		motorStop(2);	//stopping motor 2
	}
	else{
		counter[1]--;
	}
	TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
	TIM1 -> SR &= 0xfffe;// Resetting the Update Interrupt Flag (UIF) to 0
}

