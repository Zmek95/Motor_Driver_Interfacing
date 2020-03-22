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
void DCinit(void);
void motorStop(uint16_t channel);
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction);

//global variables
uint32_t counter[2]	  = {0,0};
uint8_t  timerDone[2] = {1,1};
float increment = 0; //global for now, will change to local later

void DCinit(void){
  GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
  TIM_OC_InitTypeDef  sConfig;
  TIM_HandleTypeDef tim1;
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
  motorStop(1);
  motorStop(2);
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
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_2);
  //Enable Interrupts 
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 1);//set to position 0 and priority 1
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}
void motorStop(uint16_t channel){
  if(channel == 1){	//checking channel
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

  DCinit();
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
// FUNCTION      : TIM1_UP_IRQHandler
// DESCRIPTION   : 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_UP_TIM16_IRQHandler(void) {

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

//Use another interrupt TIM1_CC
//use 2000 points, 100% PWM is at y=2.1875 for the function -140((x-1)^3)*x^3
//This function is a derivative of a SMOOTHSTEP function 
//function will operate similar to the lab 8 advanced functions
//x=1/2 is the global maximium for the function, x<1/2 accel profile, x>1/2 deccel profile
//CCR register is edited with new PWM value every cycle until it reaches desired PWM point
//To check for the desired PWM speed use greater than equality e.g for 40% duty cycle for 1000 period
//CCR1 -> 400, the function is a 1000 points at 500 100% duty cycle is reached speed desired is at (0.4*2.1875)
//quantify 2.1875 into 500 points

// FUNCTION      : TIM1_CC_IRQHandler
// DESCRIPTION   : Interrupt for controlling the PWM duty cycle for BreathingPWM for the TIM1 CC register.
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_CC_IRQHandler(void) {
	//need user entered data

	int channel;
	float y;//On startup CCR1 and CCR2 registers must be 0, y is the speed of the DC motor
	increment = increment + 0.0005;
	//checking interrupt flags
	if (TIM1->SR & TIM_SR_CC1IF) {	//Capture Compare register 1
		channel = 1;
		y = TIM1->CCR1;
		TIM1->SR &= ~TIM_SR_CC1IF;// Resetting the CC1 Interrupt Flag to 0
	}
	if (TIM1->SR & TIM_SR_CC2IF) {	//Capture Compare register 2
		channel = 2;
		y = TIM1->CCR2;
		TIM1->SR &= ~TIM_SR_CC2IF;// Resetting the CC2 Interrupt Flag to 0
	}
	
	TIM1->CR1 &= ~TIM_CR1_CEN; 	//stopping timer
	//updating duty cycle on breathing profile function based on mode selected 
	//put DC function here and pass it the duty cycle from function.
	
	//If statement here for when the desired speed is reached to skip following block
	y = -140((x-1)*(x-1)*(x-1))*x*x*x;
	y = 1000 * (y/2.1875);// Convert to a 0 to 1000 scale
	if (channel = 1){
		TIM1->CCR1 = (int) y;
	}else if (channel = 2){
		TIM1->CCR2 = (int) y;
	}
	
	
	
	TIM1->CR1 |= TIM_CR1_CEN;	//starting timer
}
