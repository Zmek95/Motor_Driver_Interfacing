/* 	ZS_lab11.c
 *	PID DC Motor controls
 * 	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *	Date: Apr 16, 2020
 */
 
#include <stdio.h>
#include <stdint.h>
#include "common.h"

#define SAMPLING_RATE 100// time in milliseconds
#define TICKS_PER_REV 198.6
#define MAX_RPM 155 //No - load speed of the motor
#define PID_START_DELAY 5 //Start delay before PID control is applied, delay is a multiple of sampling rate

//global TypeDefs
GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
TIM_OC_InitTypeDef  sConfig;
TIM_HandleTypeDef tim1;
TIM_HandleTypeDef tim3;
TIM_Encoder_InitTypeDef encoderConfig;
//global variables
uint16_t direction;//DC motor direction of rotation
uint16_t PIDStartDelay = PID_START_DELAY;//Delay before PID control is applied, time base is SAMPLING_RATE
int16_t previousPosition;//Used to determine the speed of the motor, the position of the encoder at the previous sample.
int32_t controlMeasuredSpeed;//speed measured by the encoder
int32_t desiredSpeed;//The speed entered by the user
float controlAdjustedSpeed;//Adjusted speed due to PID control
float integralError = 0;//Accumalted I error 
float previousError = 0;//error value that was determined at the previous sample
float KP = 6;//Propotional constant for P type control
float KI = 2;//Integral constant for I type control
float KD = 1;//Derivative constant for D type control

//funtion declarations
uint16_t readEncoder(void);//might do RPM conversion here
void motorStop(void);
void DC(uint16_t userSpeed, uint16_t direction);

// FUNCTION      : controlInit
// DESCRIPTION   : This function initializes the GPIOs, timers and encoders to enable PID control for the DC motor
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void controlInit(void *data){
	
	
  HAL_StatusTypeDef rc;
	
  // Enabling clock for GPIOs
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
  /*********************DC motor****************************************/
  //configure GPIO pins for Motor1
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //configuring GPIO pin for PWM output
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Making sure motor is not activated
  motorStop();
  //configuring TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = 1000;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  //Configuring enable bits for TIM1
  TIM1 -> CR1 |= TIM_CR1_URS; //only counter over/under flow generates interrupt
  TIM1 -> DIER |= TIM_DIER_CC1IE; //setting Update Interrupt Enable bit 
  TIM1 -> EGR |= TIM_EGR_UG; //setting Update Generation bit 
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
	
  /*********************Encoder****************************************/
  //configure GPIO pins	
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Enabling clock for TIM3
  __HAL_RCC_TIM3_CLK_ENABLE();
  //Configuring Timer 3
  tim3.Instance = TIM3;
  tim3.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
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
  //Initialize encoder
  rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);
  if(rc != HAL_OK) {
    printf("Failed to initialize Timer 3 Encoder, rc=%u\n",rc);
    return;
  }
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
	
  desiredSpeed = 0;
  controlMeasuredSpeed = 0;
  previousPosition = 0;
}


/*********************************************Commands & Tasks*********************************************************/


//This task measures the speed of the motor and applies PID control depending on the SAMPLING_RATE
void controlTask(void *data){
	
  uint16_t currentPosition;
  uint32_t difference;
  float errorValue;
  float derivativeError;
  float proportionalError;
  uint16_t controlAdjustedSpeedRPM;
  float TickstoRPM;
	
  //Speed measurement in ticks per 100ms
  currentPosition = readEncoder();
  difference = currentPosition - previousPosition;
  if (difference > 32768){//Protection for 65536 -> 0 overflow
    difference -= 65536;
  }
  previousPosition = currentPosition;
  controlMeasuredSpeed = difference;
	
  //A delay is applied before PID control begins
  //PID control
  if(desiredSpeed > 0 && PIDStartDelay <= 1){
    //Negative error values for overshoot, positive error values for undershoot
		
    errorValue = desiredSpeed - controlMeasuredSpeed;
		
    proportionalError = KP * errorValue;
    integralError = errorValue + integralError;
    derivativeError = KD*(errorValue - previousError);
		
    previousError = errorValue;
    errorValue = proportionalError + (KI*integralError) + derivativeError;
		
    if(errorValue != 0 && PIDStartDelay == 0){
		
      controlAdjustedSpeed = errorValue + controlAdjustedSpeed;
      //RPM conversion
      //(RawTicksValue*(60000/SAMPLING_RATE))/198.6
      TickstoRPM = (float)(controlAdjustedSpeed * (60000/SAMPLING_RATE))/TICKS_PER_REV;
      controlAdjustedSpeedRPM = (uint16_t) TickstoRPM;
      DC(controlAdjustedSpeedRPM,direction); //error needs to be converted to a duty cycle
    }else if(PIDStartDelay == 1){//decrement once after PID error is calcualted so previousError is intiallized 
      PIDStartDelay--;
    }
		
  }else{
    if(PIDStartDelay >= 0){
      PIDStartDelay--;
    }
  }
}
ADD_TASK(controlTask, controlInit, NULL, SAMPLING_RATE, "                       DC motor control task")

//This command returns the measured speed of the motor in RPM.
  ParserReturnVal_t CmdSpeed(int mode) {
	
  float TickstoRPM;
	
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
	
  TickstoRPM = (float)(controlMeasuredSpeed * (60000/SAMPLING_RATE))/TICKS_PER_REV;

  printf("Speed in RPM %.1f\n",TickstoRPM);
  return CmdReturnOk;
}
ADD_CMD("speed", CmdSpeed, "	      Prints the speed of the DC motor.")

//This command is used to set the direction and speed(RPM) of the motor.
ParserReturnVal_t CmdSetSpeed(int mode) {
  //Unit conversions placed here
  uint16_t userSpeed = 0;//used to control speed of motor (RPM)
  uint16_t userDirection = 0; 
  float RPMtoTicks;
  uint16_t rc;
  PIDStartDelay = PID_START_DELAY;
	
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
	
  rc = fetch_uint16_arg(&userDirection);	//inputting direction number from user
  if (rc) {
    printf("Please select direction of rotation (0,1,2)\n");
    return CmdReturnBadParameter1;
  }
  rc = fetch_uint16_arg(&userSpeed);	//inputting speed from user
  if (rc) {
    printf("Please enter a valid integer for the speed\n");
    return CmdReturnBadParameter2;
  }
	
  if(userDirection > 2){
    printf("Please enter a valid direction, 0 for stop, 1 for clockwise , 2 for anti-clockwise\n");
    return CmdReturnBadParameter1;
  }
  if(userSpeed > 155 || userSpeed == 0){
    printf("Please enter a valid positive integer for speed, maximum RPM is MAX_RPM\n");
    return CmdReturnBadParameter2;
  }

  direction = userDirection;
  //RPM to ticks for desiredSpeed
  RPMtoTicks = (float)(userSpeed*TICKS_PER_REV)/(60000/SAMPLING_RATE);
  desiredSpeed = (int32_t) RPMtoTicks;
  controlAdjustedSpeed = desiredSpeed;
	
  DC(userSpeed,direction);
  return CmdReturnOk;
}
ADD_CMD("setSpeed", CmdSetSpeed, "	      <direction> <speed> Sets the speed of the DC motor")

//This command is used to set the constants for PID control.
ParserReturnVal_t CmdPIDcontrol(int mode) {
  float userKP;
  float userKI;
  float userKD;
  uint16_t rc;
	
  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
	
  rc = fetch_float_arg(&userKP);	//inputting KP constant from user
  if (rc) {
    printf("Please enter a valid positive integer for PID constants\n");
    return CmdReturnBadParameter1;
  }
  rc = fetch_float_arg(&userKI);	//inputting KI constant from user
  if (rc) {
    printf("Please enter a valid positive integer for PID constants\n");
    return CmdReturnBadParameter1;
  }
  rc = fetch_float_arg(&userKD);	//inputting KD constant from user
  if (rc) {
    printf("Please enter a valid positive integer for PID constants\n");
    return CmdReturnBadParameter1;
  }
	
  KP = userKP;
  KI = userKI;
  KD = userKD;
	
  return CmdReturnOk;
}
ADD_CMD("PIDcontrol", CmdPIDcontrol, "	       <KP> <KI> <KD> Change the constants for PID control.")


//DC motor functions

// FUNCTION      : motorStop()
// DESCRIPTION   : This function stops the motor
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void motorStop(void){

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //creating zero potential between motor 1 terminals
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
}

// FUNCTION      : DC()
// DESCRIPTION   : Sets the duty cycle for the PWM waveform for the motor
// PARAMETERS    : uint16_t dutyCycle - duty cycle for the PWM waveform
//                 uint16_t direction - direction of rotation
// RETURNS       : Nothing
void DC(uint16_t userSpeed, uint16_t direction){//RPM to dutyCycle conversion in this function
	
  float dutyCycle;
	
  dutyCycle =((float)userSpeed/MAX_RPM)*100;
  dutyCycle = dutyCycle * 10;	//scaling up so that 100% duty cycle corresponds to 1000 (pulse = period = 1000)
  TIM1->CR1 &= ~TIM_CR1_CEN;	//stopping timer
  
  TIM1->CCR1 = (uint16_t) dutyCycle;              //setting new duty cycle 
  TIM1->CCER &= ~(TIM_CCER_CC1P);	     //enabling channel output
  TIM1->CCER |= TIM_CCER_CC1E;
	
  if(direction == 1){//Clockwise
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  }else if(direction == 2){//Anti - Clockwise
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  }else if(direction == 0){// Stop
    motorStop();
  }
  
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
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
  
