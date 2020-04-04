/*	File name: ZS_lab10.c
 *
 *	Program Desription: This program reads and controls the postion of a DC motor using encoder feedback signal and internal timer in encoder mode
 *	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *
 *	Date: Mar 31, 2020 */

#include <stdio.h>
#include <stdint.h>
#include "common.h"

//prototypes
void encoderInit(void);
uint16_t readEncoder(void);
void resetPosition(void);
void runMotor(uint16_t position, uint16_t direction);
void motorStop(uint16_t channel);
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction);

//TypeDefs
TIM_HandleTypeDef tim3;
TIM_OC_InitTypeDef  sConfig;

/********************************Functions*******************************/
// FUNCTION      : encoderInit()
// DESCRIPTION   : Intializes the GPIO pins, timer 3 channels 1 & 2 in encoder mode, channel 3 in output compare mode with interrupt
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void encoderInit(void) {
  HAL_StatusTypeDef rc;
  GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
  TIM_Encoder_InitTypeDef encoderConfig;
  // Enabling clock for GPIOs
  __HAL_RCC_GPIOA_CLK_ENABLE();
  //configure GPIO pins	
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Enabling clock for TIM3
  __HAL_RCC_TIM3_CLK_ENABLE();
  //Configuring channel 3
  TIM3 -> DIER = 0X0008; //enabling interrupt for CC3 channel
  TIM3 -> CCMR2 = 0;     //configuring CC3 channel as output
  TIM3 -> CCER = 0X0100; //enabling output on CC3 channel
  TIM3 -> CR1 |= 0X0002; //Update event disabled
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
  //Configuring encoder
  encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity = 0;
  encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler = 0;
  encoderConfig.IC1Filter = 3;
  encoderConfig.IC2Polarity = 0;
  encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler = 0;
  encoderConfig.IC2Filter = 3;
  rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);   //Initialize encoder
  if(rc != HAL_OK) {
    printf("Failed to initialize Timer 3 Encoder, rc=%u\n",rc);
    return;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1);   //Start Encoder
  if(rc != HAL_OK) {
    printf("Failed to start Timer 3 Encoder channel 1, rc=%u\n",rc);
    return;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2);
  if(rc != HAL_OK) {
    printf("Failed to start Timer 3 Encoder channel 2 , rc=%u\n",rc);
    return;
  }
  HAL_NVIC_SetPriority(TIM3_IRQn,2,3);
  NVIC_EnableIRQ(TIM3_IRQn);
}

// FUNCTION      : readEncoder()
// DESCRIPTION   : Reads the current value of the quadrature encoder to determine the position of the motor
// PARAMETERS    : Nothing
// RETURNS       : uint16_t encoderValue : Value of timer 3 counter which corresponds to motor's current position
uint16_t readEncoder(void){
  uint16_t encoderValue;
  encoderValue = TIM3->CNT;
  return encoderValue;
}
// FUNCTION      : resetPosition()
// DESCRIPTION   : Resets the quadrature encoder to begin counting from zero. 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void resetPosition(void){
  motorStop(1);     //make sure motor is stopped
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0); 			//used for simulation purpose only
 TIM3->CNT = 0;
}
// FUNCTION      : runMotor()
// DESCRIPTION   : Moves the motor to a specified relative position then stops the motor 
// PARAMETERS    : uint16_t position: The position the motor will be moved to
//		   uint16_t direction: The direction the motor shaft will move.
// RETURNS       : Nothing
void runMotor(uint16_t position, uint16_t direction){
  motorStop(1);                       //make sure motor is stopped
  TIM3->CR1 &= ~TIM_CR1_CEN;          //stopping timer 3
  TIM3->CCR3 = position;              //copying position to Campture Compare register
  TIM3->CR1 |= TIM_CR1_CEN;                //resuming  timer 3
  DC(1,50,direction);                      //starting motor with specified direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,1);  //used for simulation purpose only 
}
/****************************Commands***********************/
ParserReturnVal_t CmdEncoderInit(int mode) {
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  encoderInit(); 	                                //Initializes DC motor, and configure timer 3 in encoder mode
  return CmdReturnOk;
}
ADD_CMD("EncoderInit", CmdEncoderInit, "			Initializes DC motor, and configure timer 3 in encoder mode")

ParserReturnVal_t CmdreadEncoder(int mode) {
  uint16_t rawValue = 0;   		                //position in raw ticks
  float positionInDegrees = 0;                          //position in degrees
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  rawValue = readEncoder();	                        //Reads the current value from the quadrature encoder
  positionInDegrees = (float)rawValue * 360 / 198.6;	//converting into degrees
  printf("Current position: %f degrees\n", positionInDegrees);
  return CmdReturnOk;
}
ADD_CMD("readEncoder", CmdreadEncoder, "			prints out current position of motor in degrees")

ParserReturnVal_t CmdresetPosition(int mode) {
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  resetPosition();//sets current position as zero (reference point)
  return CmdReturnOk;
}
ADD_CMD("resetPosition", CmdresetPosition, "			sets current position as zero (reference point)")

ParserReturnVal_t CmdstopPosition(int mode) {
  float positionInDegrees = 0;                   //position in degrees
  uint32_t rawValue = 0;   		         //position in raw ticks
  uint16_t rc;
  int direction;				 //direction of rotation
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  rc = fetch_float_arg(&positionInDegrees);	//inputting position from user
  if (rc) {
    printf("Please select position in degrees\n");
    return CmdReturnBadParameter1;
  }
  if(positionInDegrees > 118794 || positionInDegrees < -118794){     //making sure input doesn't overflow counter
    printf("position must be between +/- 118794\n");
    return CmdReturnBadParameter1;
  }
  if (positionInDegrees < 0){			  //extracting direction from sign
    positionInDegrees = -positionInDegrees;
    direction = 1;
  }
  else{
    direction = 0;
  }
  rawValue = positionInDegrees * 198.6 / 360;     //converts into raw ticks
  runMotor(rawValue, direction);		  //configures timer and motor for operation
  return CmdReturnOk;
}
ADD_CMD("stopPosition", CmdstopPosition, "			sets desired position (in degrees) for motor to stop at")
/*****************************ISR*******************************************/
// FUNCTION      : TIM3_IRQHandler()
// DESCRIPTION   : Stops motor 1 when timer 3 counter matches output compare channel 3 register
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM3_IRQHandler(void){
  TIM3->CR1 &= ~TIM_CR1_CEN; 	   //stopping timer3
  //checking interrupt flags
  if (TIM3->SR & TIM_SR_CC3IF) {   //Capture Compare register 3
    motorStop(1);		   //stopping motor 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0); 			//used for simulation purpose only     
    TIM3->SR &= ~TIM_SR_CC3IF;     // Resetting the CC3 Interrupt Flag to 0
  }	
  TIM3->CR1 |= TIM_CR1_CEN; 	   //resuming timer3
}
