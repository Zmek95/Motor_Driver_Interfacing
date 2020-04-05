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
void runMotor(uint16_t position);
void DCmotorStop(void);
void DCM_Encoder(void);

//global typeDefs
TIM_OC_InitTypeDef  sConfig;
TIM_HandleTypeDef tim1;
TIM_HandleTypeDef tim3;
HAL_StatusTypeDef rc;
GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
TIM_Encoder_InitTypeDef encoderConfig;

//global variables
static int direction = 0;	//direction of rotation
static uint16_t tempCNT;	//used to temporarily hold the value of CNT register
static uint16_t counter = 0;		//used for position values greater than 65536
static uint16_t remaining = 0;		//represents the remaining portion of total counts
/********************************Functions*******************************/
// FUNCTION      : encoderInit()
// DESCRIPTION   : Intializes the GPIO pins, timer 3 channels 1 & 2 in encoder mode, channel 3 in output compare mode with interrupt, timer 1 channel in PWM mode
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void encoderInit(void) { 
 
  // Enabling clock for GPIOs
  __HAL_RCC_GPIOA_CLK_ENABLE();
  //configure GPIO pins	for signals A and B
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //configure GPIO pins for Motor1 direction
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //configuring GPIO pins for PWM outputs for motor 1
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Enabling clock for TIM3
  __HAL_RCC_TIM3_CLK_ENABLE();
  //Configuring channel 3
  TIM3 -> DIER = 0X0008; //enabling interrupt for CC3 channel
  TIM3 -> CCMR2 = 0;     //configuring CC3 channel as output
  TIM3 -> CR1 |= 0X0002; //Update event disabled
  //Configuring Timer 3
  tim3.Instance = TIM3;
  tim3.Init.Prescaler = 0;
  tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim3.Init.Period = 0xffff;
  tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim3.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&tim3);
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
  //configuring TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = 10000;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&tim1);
  //Configuring PWM
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = 5000;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
  TIM1->CCER &= 0xFFFFFFFE;	//disabling PWM output
  TIM1->CCER |= 0x04;
  //setting interrupt priority
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
  DCmotorStop();           //stopping motor first
  TIM3->CNT = 0;	   // resetting counter
}

// FUNCTION      : runMotor()
// DESCRIPTION   : Moves the motor to a specified relative position in a specified direction
// PARAMETERS    : uint16_t position: position of desired destination
// RETURNS       : Nothing
void runMotor(uint16_t position){

  DCmotorStop();                 //make sure motor is stopped first
  TIM3->CR1 &= ~TIM_CR1_CEN; 	 //stopping timer3
  //checking direction
  if(direction == 1){
    tempCNT = TIM3->CNT;	//temporarily storing value of CNT
    //flipping direction of counting mode
    tim3.Instance = TIM3;
    tim3.Init.Prescaler = 0;
    tim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    tim3.Init.Period = 0xffff;
    tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim3.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&tim3);
    HAL_TIM_Encoder_Init(&tim3, &encoderConfig);   //Initialize encoder
    TIM3->CNT = tempCNT;			   //restoring back value of CNT
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  }else{
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
   }
  TIM3->CCR3 = position;                       //copying position to Campture Compare register
  HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1); //Start Encoder channel 1
  HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2); //Start Encoder channel 2
  TIM1->CCER &= 0xFFFFFFFC;	     	       //enabling PWM output
  TIM1->CCER |= 0x01; 
}
// FUNCTION      : DCmotorStop()
// DESCRIPTION   : This function stops the motor by creating zero potential between motor terminals
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void DCmotorStop(void){
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    TIM1->CCER &= 0xFFFFFFFE;	//disabling PWM output
    TIM1->CCER |= 0x04;
}
/****************************Commands***********************/
ParserReturnVal_t CmdEncoderInit(int mode) {
  
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  encoderInit(); 	                                //Initializes DC motor, and configure timer 3 in encoder mode
  return CmdReturnOk;
}
ADD_CMD("EncoderInit", CmdEncoderInit, "			Initializes DC motor, and configure timer 3 in encoder mode")
/***********************************************************/
ParserReturnVal_t CmdreadEncoder(int mode) {
  
  uint32_t rawValue = 0;   		                //position in raw ticks
  uint32_t positionInDegrees = 0;                          //position in degrees
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  rawValue = readEncoder();	                        //Reads the current value from the quadrature encoder
  positionInDegrees = rawValue * 360 / 198.6;	//converting into degrees
  printf("Current position: %lu degrees\n", positionInDegrees);
  return CmdReturnOk;
}
ADD_CMD("readEncoder", CmdreadEncoder, "			prints out current position of motor in degrees")
/***********************************************************/
ParserReturnVal_t CmdresetPosition(int mode) {
  
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  resetPosition();//sets current position as zero (reference point)
  return CmdReturnOk;
}
ADD_CMD("resetPosition", CmdresetPosition, "			sets current position as zero (reference point)")
/***********************************************************/
ParserReturnVal_t CmdstopPosition(int mode) {
  
  int32_t positionInDegrees = 0;                   //position in degrees
  uint16_t rawValue = 0;   		         //position in raw ticks
  uint16_t rc;

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  rc = fetch_int32_arg(&positionInDegrees);	//inputting position from user
  if (rc) {
    printf("Please select position in degrees\n");
    return CmdReturnBadParameter1;
  }
  if (positionInDegrees < 0){			  //extracting direction from sign
    positionInDegrees = -positionInDegrees;
    direction = 1;
  }
  else{
    direction = 0;
  }
  if(positionInDegrees >= 182){
    counter = positionInDegrees / 182;
    remaining = positionInDegrees % 182;
    remaining *= 198.6 / 360;
    rawValue = 0xFFFF;
  }else{
    rawValue = positionInDegrees * 198.6 / 360;
  }
  runMotor(rawValue);		  //configures timer and motor for operation
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
    if(counter == 0){
      DCmotorStop();		   //stopping motor
      if( direction == 1){
        tempCNT = TIM3->CNT;	//temporarily storing value of CNT
        //flipping direction of counting mode
        tim3.Instance = TIM3;
        tim3.Init.Prescaler = 0;
        tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
        tim3.Init.Period = 0xffff;
        tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        tim3.Init.RepetitionCounter = 0;
        HAL_TIM_Base_Init(&tim3);
        HAL_TIM_Encoder_Init(&tim3, &encoderConfig);   //Initialize encoder
        TIM3->CNT = tempCNT;	//restoring back value of CNT
      }
    }else if(counter == 1){
      TIM3->CCR3 = remaining;
      counter--;
    }else{
      TIM3->CCR3 = 0xFFFF;
      counter--;
    }
    TIM3->SR &= ~TIM_SR_CC3IF;     // Resetting the CC3 Interrupt Flag to 0
  }	
}
