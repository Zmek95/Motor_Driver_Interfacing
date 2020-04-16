/* 	ZS_lab11.c
 *	PID DC Motor controls
 * 	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *	Date: Apr 15, 2020
 */
 
 /* Remaining tasks
  * 1. D term needs to be sampled over a larger amount of points to reduce noise,
  *    so a linear regression needs to be done over a few error values (e.g 10 previous 
  *    errors) to calculate the derivative.
  */
 
#include <stdio.h>
#include <stdint.h>
#include "common.h"

#define SAMPLING_RATE 100
#define KP 6 //Propotional constant for P type control
#define KI 2 //Integral constant for I type control
#define KD 1 //Derivative constant for D type control
#define TICKS_PER_REV 198.6
#define MAX_RPM 155 //No - load speed of the motor

//global TypeDefs
GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
TIM_OC_InitTypeDef  sConfig;
TIM_HandleTypeDef tim1;
TIM_HandleTypeDef tim3;
TIM_Encoder_InitTypeDef encoderConfig;
//global variables
int32_t controlMeasuredSpeed;
int16_t previousPosition;
int32_t desiredSpeed;
float controlAdjustedSpeed;
int PIDStartDelay = 10;
uint16_t direction;
float integralError = 0;
float previousError = 0;

//funtion declarations
uint16_t readEncoder(void);//might do RPM conversion here
void motorStop(void);
void DC(uint16_t userSpeed, uint16_t direction);


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
	//configuring GPIO pins for PWM outputs
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
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
	
	//allow for 1 second to pass before PID control is applied
	//PID control
	if(desiredSpeed > 0 && PIDStartDelay == 1){
		//Negative error values for overshoot, positive error values for undershoot
		errorValue = desiredSpeed - controlMeasuredSpeed;
		
		proportionalError = KP * errorValue;
		integralError = errorValue + integralError;
		derivativeError = KD*(errorValue - previousError);
		
		previousError = errorValue;
		errorValue = proportionalError + (KI*integralError) + derivativeError;
		
		if(errorValue != 0 && PIDStartDelay == 0){
			
			if (errorValue < 0){
				//reverse motor direction!!!
				if(direction == 0){//reverse direction
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				}else{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
				}
			}
			//RPM conversion here
			//(RawTicksValue*(60000/SAMPLING_RATE))/198.6
			TickstoRPM = (float)(errorValue * (60000/SAMPLING_RATE))/TICKS_PER_REV;
			controlAdjustedSpeedRPM = (uint16_t) TickstoRPM;
			DC(controlAdjustedSpeedRPM,direction); //error needs to be converted to a duty cycle
		}
	}else{
		if(PIDStartDelay >= 0){
			PIDStartDelay--;
		}
	}
	
	
}
ADD_TASK(controlTask, controlInit, NULL, SAMPLING_RATE, "DC motor control task")

ParserReturnVal_t CmdSpeed(int mode) {
	
	float TickstoRPM;
	
	if (mode != CMD_INTERACTIVE) return CmdReturnOk;
	
	TickstoRPM = (float)(controlMeasuredSpeed * (60000/SAMPLING_RATE))/TICKS_PER_REV;

	printf("Speed in RPM %.1f\n",TickstoRPM);
	return CmdReturnOk;
}
ADD_CMD("speed", CmdSpeed, "	      Prints the speed of the DC motor.")

ParserReturnVal_t CmdSetSpeed(int mode) {
	//Unit conversions placed here
	uint16_t userSpeed = 0;//used to control speed of motor (RPM)
	float RPMtoTicks;
	uint16_t rc;
	PIDStartDelay = 10;
	
	if (mode != CMD_INTERACTIVE) return CmdReturnOk;
	
	rc = fetch_uint16_arg(&direction);	//inputting direction number from user
	if (rc) {
		printf("Please select direction of rotation\n");
		return CmdReturnBadParameter1;
	}
	rc = fetch_uint16_arg(&userSpeed);	//inputting speed from user
    if (rc) {
		printf("Please enter the speed, (0 to 100)\n");
		return CmdReturnBadParameter2;
    }

	//RPM to ticks for desiredSpeed
	RPMtoTicks = (float)(userSpeed*TICKS_PER_REV)/(60000/SAMPLING_RATE);
	desiredSpeed = (int32_t) RPMtoTicks;
	
	DC(userSpeed,direction);
	return CmdReturnOk;
}
ADD_CMD("setSpeed", CmdSetSpeed, "	      Prints the speed of the DC motor.")





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
	//checking channel
  
	TIM1->CCR1 = (uint16_t) dutyCycle;              //setting new duty cycle for channel 1 
	TIM1->CCER &= 0xFFFFFFFC;	     //enabling channel1 output
	TIM1->CCER |= 0x01;
	if(direction == 1){              //checking direction
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
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
  
