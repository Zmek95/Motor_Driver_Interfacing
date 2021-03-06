/* 	ZS_lab9.c
 *	DC Motor controls
 * 	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *	Date: Mar 19, 2020
 */
#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "DC_CommandQueue.h"
//defines
#define INCREMENT 0.0005 //The increment along the function -140((x-1)^3)*x^3 from x= 0 to x = 1 (2000 points in this case)
#define MAXIMUM 2.1875   //The global maximum of the function -140((x-1)^3)*x^3 used in the speedProfile function 
#define MINIMUM 0.027    //This is the lowest value for increment which results to a CCR value of 1 after calculations
#define PERIOD 1000      //Speed starts to change when time left equals PERIOD (1000 ms)
//prototypes
void DCinit(void);
void motorStop(uint16_t channel);
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction);
void speedProfile(uint16_t channel,int newDutyCycleFlag);
void checkQueue(void);
//global variables
uint32_t counter[2] = {0,0};            //used to store number of overflows left for timer
uint8_t  timerDone[2] = {1,1};          //flag used to indicate if timer is running during an motor operation
float increment[2] = {MINIMUM,MINIMUM} ;//used to store the current speed for the speedProfile function
uint16_t dutyCycleProfile[2];           //Stores the current duty cycle
extern uint16_t queueCounter[2];        //counts number of commands for each channel
/**********************************Functions**********************************/
// FUNCTION      : DCinit()
// DESCRIPTION   : This function initializes the GPIOs, timer1 channels 1 & 2 as PWM, stops motors, and enables interrupt for overflow and capture/compare
// PARAMETERS    : Nothing
// RETURNS       : Nothing
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
  //configuring GPIO pins for PWM outputs
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Making sure motors are not activated
  motorStop(1);
  motorStop(2);
  //configuring TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = PERIOD;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  //Configuring enable bits for TIM1
  TIM1 -> CR1 |= (TIM_CR1_URS); //only counter over/under flow generates interrupt
  TIM1 -> DIER |= 0b1; //setting Update Interrupt Enable bit 
  TIM1 -> EGR |= 0b1; //setting Update Generation bit 
  HAL_TIM_Base_Init(&tim1);
  //Configuring PWM
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = 0;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
  //Enable Interrupts 
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 1);//set to position 0 and priority 1
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 2);
  NVIC_EnableIRQ(TIM1_CC_IRQn);
}
// FUNCTION      : motorStop()
// DESCRIPTION   : This function stops a particular motor based on channel number passed
// PARAMETERS    : uint16_t channel - represents the motor needed to be stopped
// RETURNS       : Nothing
void motorStop(uint16_t channel){
  if(channel == 1){	//checking channel
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //creating zero potential between motor 1 terminals
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  }
  else{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0); //creating zero potential between motor 2 terminals
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
  }
}
// FUNCTION      : DC()
// DESCRIPTION   : This function sets the duty cycle for the PWM waveform for a specified channel, and controls the rotational direction of the specified motor
// PARAMETERS    : uint16_t channel - 2 channels that can generate PWM waveforms independently
//                 uint16_t dutyCycle - duty cycle for the PWM waveform
//		   uint16_t direction - direction of rotation
// RETURNS       : Nothing
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction){
	
  dutyCycle = dutyCycle * 10;	//scaling up so that 100% duty cycle corresponds to 1000 (pulse = period = 1000)
  TIM1->CR1 &= ~TIM_CR1_CEN;	//stopping timer
  //checking channel
  if (channel == 1) {
    dutyCycleProfile[0] = dutyCycle; //copying current duty cycle to use later in speed profile
    speedProfile(1,1);               //setting new duty cycle for channel 1 for smooth speed change
    TIM1->CCER &= 0xFFFFFFFC;	     //enabling channel1 output
    TIM1->CCER |= 0x01;
    if(direction == 1){              //checking direction
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    }else{
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
    }
  }
  else if(channel == 2){
    dutyCycleProfile[1] = dutyCycle; //copying current duty cycle to use later in speed profile
    speedProfile(2,1);               //setting new duty cycle for channel 1 for smooth speed change
    TIM1->CCER &= 0xFFFFFFCF;	     //enabling channel2 output
    TIM1->CCER |= 0x10;
    if(direction == 1){              //checking direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
    }else{
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
    } 
  }
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
}
// FUNCTION      : speedProfile()
// DESCRIPTION   : This function applies a speed profile based on a waveform y = (-140/2.1875)*(x-1)^3 * x^3, where 0 <= x <= 1 for a specified duty cycle
// PARAMETERS    : uint16_t channel - 2 channels that can generate PWM waveforms independently
//                 int newDutyCycleFlag - This flag is for checking whether a new duty cycle has been entered by the user
// RETURNS       : Nothing
void speedProfile(uint16_t channel,int newDutyCycleFlag){

  float y;      //The final calculated speed of the DC motor
  float x;      //Represents the increment in x for the function -140((x-1)^3)*x^3 which is the derivative of a S3 SMOOTHSTEP function 
  int newSpeed; //used to store the new speed after calculation 	

  //This block is for varying the motor speed before it has come to a stop and a new duty cycle has been entered.
  //The function used is symmetrcial over the x = 1/2 axis for 0 <= x <= 1 
  if (newDutyCycleFlag == 1){
    if (channel == 1){
      TIM1->DIER |= TIM_DIER_CC1IE;
      if(TIM1->CCR1 > dutyCycleProfile[0]){  //Check if new speed is less than current speed
	if(increment[0] < 0.5){
	  increment[0] = 1 - increment[0];  //For the function -140((x-1)^3)*x^3 when 0<x<0.5 speed increases, when 0.5<x<1 speed decreases.
	}                                   //When x = 0 or x = 1 speed is zero, and maximum speed is at x = 0.5
      }else if(TIM1->CCR1 < dutyCycleProfile[0]){
	if(increment[0] > 0.5){
	  increment[0] = 1 - increment[0];
	}
      }
    }else if(channel == 2){
      TIM1->DIER |= TIM_DIER_CC2IE;
      if(TIM1->CCR2 > dutyCycleProfile[1]){
	if(increment[1] < 0.5){
	  increment[1] = 1 - increment[1];
	}
      }else if(TIM1->CCR2 < dutyCycleProfile[1]){
	if(increment[1] > 0.5){
	  increment[1] = 1 - increment[1];
	}
      }			
    }
  }	
  //This block contains the calculations for the speed profile
  if (channel == 1){
    if ((counter[0] < PERIOD) && (increment[0] < 0.5)){  //Start slowing down the motor when counter is at the last period
      increment[0] = (1 - increment[0]) + INCREMENT;     //Shift x to be greater than 0.5 so incrementing decreases speed
      TIM1->DIER |= TIM_DIER_CC1IE;
    }else if(TIM1->CCR1 >= (dutyCycleProfile[0] - 10) && TIM1->CCR1 <= (dutyCycleProfile[0] + 10) && counter [0] > PERIOD){
      TIM1->CCR1 = dutyCycleProfile[0]; //do nothing because the desired speed has been reached within a 1% error margin
    }else{
      increment[0] = increment[0] + INCREMENT;
      x = increment[0];
      y = -140*((x-1)*(x-1)*(x-1))*x*x*x;
      y = PERIOD * (y/MAXIMUM);   //scaling to 1000 for a period of a 1000
      newSpeed = (int) y;        //copying result   
      TIM1->CCR1 = newSpeed;
      //Check to stop motor 1
      if (increment[0] >= (1-MINIMUM)){
	motorStop(1);
	increment[0] = MINIMUM;
	TIM1->DIER &= ~TIM_DIER_CC1IE;
      }
    }
  }else if (channel == 2){
    if ((counter[1] < PERIOD) && (increment[1] < 0.5)){
      increment[1] = (1 - increment[1]) + INCREMENT;
      TIM1->DIER |= TIM_DIER_CC2IE;
    }else if(TIM1->CCR2 >= (dutyCycleProfile[1] - 10) && TIM1->CCR1 <= (dutyCycleProfile[1] + 10) && counter [1] > PERIOD){
      TIM1->CCR2 = dutyCycleProfile[1]; //do nothing because the desired speed has been reached within a 1% error margin
    }else{
      increment[1] = increment[1] + INCREMENT;
      x = increment[1];
      y = -140*((x-1)*(x-1)*(x-1))*x*x*x;
      y = PERIOD * (y/MAXIMUM);   //scaling to 1000 for a period of a 1000
      newSpeed = (int) y;
      TIM1->CCR2 = newSpeed;
      //Check to stop motor 2
      if (increment[1] >= (1-MINIMUM)){
	motorStop(2);
	increment[1] = MINIMUM;
	TIM1->DIER &= ~TIM_DIER_CC2IE;
      }
    }
  }
}
// FUNCTION      : checkQueue()
// DESCRIPTION   : This function checks if queue has pending commands
// PARAMETERS    : uint16_t channel - 2 channels that can generate PWM waveforms independently
//                 uint16_t dutyCycle - duty cycle for the PWM waveform
//		   uint16_t direction - direction of rotation
// RETURNS       : Nothing
void checkQueue(void){
	struct queue data;    // used to point to the node in the queue that has the data that will be read.
	for(int i=0;i<2;i++){ // checking both channels for pending commands
	        //checking if queue is not empty and channel is not busy, and compensating for time needed for speed change
		if(queueCounter[i] > 0 && ((counter[i] < (PERIOD + 100)) || (timerDone[i] == 1))){ 
			if (timerDone[i] == 1){
				timerDone[i] = 0;      //setting flag for channel as busy
			}
			data = extractFromQueue(i+1);   //extracting the first node's data from queue
			if(data.direction == 0){
				motorStop(i+1);		//stopping motor
			}else{
				counter[i] = data.time; //calculating the number of overflows needed
				DC(data.channel, data.dutyCycle, data.direction); //setting channel, duty cycle, and direction
			}	
    		}
  	}	
}
/************************Commands*******************************/
ParserReturnVal_t CmdDCInit(int mode) {

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  DCinit();
  return CmdReturnOk;
}
ADD_CMD("DCInit", CmdDCInit, "	      Initializes DC motor driver, TIM1 channels 1 & 2 as PWM, and enables interrupt")

ParserReturnVal_t CmdDC(int mode) {

  uint16_t channel = 0;   //used to choose which motor to be activated
  uint16_t dutyCycle = 0; //used to control speed of motor
  uint16_t direction = 0; //used to set direction of rotation
  uint32_t time = 0;	  //time in milliseconds
  uint16_t rc;

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;

  rc = fetch_uint16_arg(&channel);	//inputting channel number from user
  if (rc) {
    printf("Please select which motor to control\n");
    return CmdReturnBadParameter1;
  }
  rc = fetch_uint16_arg(&direction);	//inputting direction number from user
  if (rc) {
    printf("Please select direction of rotation\n");
    return CmdReturnBadParameter3;
  }
  if(direction == 1 || direction == 2){ //checking if break is not requested
    rc = fetch_uint32_arg(&time);	//inputting direction number from user
    if (rc) {
      printf("Please enter time in milliseconds\n");
      return CmdReturnBadParameter4;
    }
    rc = fetch_uint16_arg(&dutyCycle);	//inputting speed from user
    if (rc) {
      printf("Please enter the speed, (0 to 100)\n");
      return CmdReturnBadParameter2;
    }
  } 
  if (channel > 2 || channel < 1) {	//checking if channel number is valid 
    printf("Type either 1 or 2 for motor selection\n");
    return CmdReturnBadParameter1;
  }
  if (direction > 2 || direction < 0) {	//checking if direction is valid 
    printf("Type 1 for CW, 2 for CCW or 0 to stop motor\n");
    return CmdReturnBadParameter3;
  }
  if (time < 0 || time > 30000) {	//checking if time is valid 
    printf("time can't be negative or greater than 30000 ms\n");
    return CmdReturnBadParameter4;
  }
  if (dutyCycle > 100 || dutyCycle <= 0) {	//checking if speed selected is valid 
    printf("speed must be between 0 and 100\n");
    return CmdReturnBadParameter2;
  }
  addToQueue(channel, dutyCycle, direction, time);
  return CmdReturnOk;
}
ADD_CMD("DC", CmdDC, "<channel><direction><time><speed>  activates selected motor and sets direction, speed and operating time")
/***************************ISR****************************/
// FUNCTION      : TIM1_UP_IRQHandler
// DESCRIPTION   : This ISR is visited everytime TIM1 overflows. counter[] is used to track the remaining time
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_UP_TIM16_IRQHandler(void) {
  
  TIM1->CR1 &= ~TIM_CR1_CEN; 	//stopping timer
  if(counter[0] == 0){          //checking if channel 1 timed-out
    motorStop(1); 	        //stopping motor 1
    TIM1->CCER &= 0xFFFFFFFE;	//disabling channel 1 output
    TIM1->CCER |= 0x04;
    timerDone[0] = 1;           //flag raised to indicate that channel 1 is not busy
  }
  else{
    counter[0]--;
  }	
  if(counter[1] == 0){          //checking if channel 2 timed-out
    motorStop(2);	        //stopping motor 2
    TIM1->CCER &= 0xFFFFFFEF;	//disabling channel 2 output
    TIM1->CCER |= 0x40;
    timerDone[1] = 1;           //flag raised to indicate that channel 2 is not busy
  }
  else{
    counter[1]--;
  }
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
  TIM1 -> SR &= 0xfffe;         // Resetting the Update Interrupt Flag (UIF) to 0
}
// FUNCTION      : TIM1_CC_IRQHandler
// DESCRIPTION   : Interrupt that executes the speedProfile function everytime the counter compare register matches the counter.
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_CC_IRQHandler(void) {
  TIM1->CR1 &= ~TIM_CR1_CEN; 	//stopping timer
  //checking interrupt flags
  if (TIM1->SR & TIM_SR_CC1IF) {	//Capture Compare register 1
    speedProfile(1,0);
    TIM1->SR &= ~TIM_SR_CC1IF;// Resetting the CC1 Interrupt Flag to 0
  }
  if (TIM1->SR & TIM_SR_CC2IF) {	//Capture Compare register 2
    speedProfile(2,0);
    TIM1->SR &= ~TIM_SR_CC2IF;// Resetting the CC2 Interrupt Flag to 0
  }	
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
}
