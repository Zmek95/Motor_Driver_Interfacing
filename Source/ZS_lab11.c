/* 	ZS_lab11.c: Closed Loop Control
 *	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *  	Date: Apr 12, 2020
 */
#include <stdio.h>
#include <stdint.h>
#include "common.h"

//TypeDefs
TIM_HandleTypeDef tim1;
TIM_OC_InitTypeDef  sConfig;
GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
ADC_HandleTypeDef hadc;

//prototypes
void controlInit(void *data);
void HD44780_Init(void);
void HD44780_GotoXY(unsigned char x, unsigned char y);
void HD44780_PutStr(char *str);

//global variables
char stringBuffer[16]= {0};     	    //buffer used to store different datatypes for LCD
volatile uint32_t controlMeasuredSpeed = 0; //used to hold the measured output value
uint32_t desiredSpeed = 0;		    //holds the desired setpoint value
int Kp = 1;				    //proportional constant
int Kd = 4;				    //differential constant
int32_t proportionalError = 0;		
int32_t differentialError = 0;
int32_t previousError = 0;		    //holds the error from previous sample

/*************Commands****************************/

ParserReturnVal_t CmdSetInput(int mode) {

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  fetch_uint32_arg(&desiredSpeed);		//inputting desired speed
  TIM1->CR1 &= ~TIM_CR1_CEN;    		//stopping timer	
  TIM1->CCR1 = desiredSpeed * 65500 / 1738;	//setting desired speed as input
  TIM1->CR1 |= TIM_CR1_CEN;			//resuming timer
  HAL_ADC_Start_IT(&hadc);			//starting ADC with interrupt
  return CmdReturnOk;
}
ADD_CMD("SetInput", CmdSetInput, "		Set desired speed (range: 0 to 1738)")

/*******************Tasks**************************/
// TASK          : controlTask()
// DESCRIPTION   : Takes a sample of output every 10 ms, and displays the output on LCD every 750 ms
// PARAMETERS    : void *data
// RETURNS       : Nothing
void controlTask(void *data){
	
  static int LCDDelay = 0;    //used as counter to set delay between LDC displays
  uint16_t outputPercentage;  //represents the percentage of output with respect to desired set point

    HAL_ADC_Start_IT(&hadc); //taking new sample from output 

  //display results on LCD every 750 ms
  if(LCDDelay % 75 == 0){
    outputPercentage = controlMeasuredSpeed * 100 / desiredSpeed ; //calculating percentage of output with respect to desired set point
    //displaying output on LCD line 1
    HD44780_GotoXY(0,0);	
    sprintf(stringBuffer,"%03d prcnt stp",outputPercentage);
    HD44780_PutStr(stringBuffer);
    //displaying current duty cycle on LCD line 2      	                
    HD44780_GotoXY(0,1);	
    sprintf(stringBuffer,"@ %03lu dutyCyc",TIM1->CCR1 * 100 / 65500);
    HD44780_PutStr(stringBuffer);			
  }
  LCDDelay++;
}
ADD_TASK(controlTask, controlInit, NULL, 10, "DC motor control task")


/***************************ISR****************************/
// FUNCTION      : HAL_ADC_ConvCpltCallback()
// DESCRIPTION   : Reads the converted sample, calculates and updates the new duty cycle using PD control
// PARAMETERS    : ADC_HandleTypeDef* hadc1 - pointer to ADC1 peripheral structure
// RETURNS       : Nothing
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
    {   
	int32_t newValue = 0;    //used to update the duty cycle
	int32_t errorValue = 0;  //represents the difference between the input and output values

	if(hadc1->Instance == ADC1){

	  //PID controller code
  	  controlMeasuredSpeed = HAL_ADC_GetValue(&hadc);   //copying result of conversion

	  //calculating proportional and differential errors
	  errorValue = desiredSpeed - controlMeasuredSpeed; 
	  proportionalError = errorValue;	            
	  differentialError = errorValue - previousError;   

	  previousError = errorValue;			    //updating previous error for next sample

	  //calculating the new duty cycle
	  newValue = (proportionalError * Kp) + (differentialError * Kd) + TIM1->CCR1 * 1738 / 65500;
	  
	  //setting limits for the new duty cycle governed by system's hardware
	  if(newValue > 1738){
	    newValue = 1738;
	  }else if(newValue < 0){
	    newValue = 0;
	  }
	    TIM1->CR1 &= ~TIM_CR1_CEN;  	  //stopping timer	
	    TIM1->CCR1 =  newValue * 65500 / 1738;//updating duty cyle
	    TIM1->CR1 |= TIM_CR1_CEN;	          //resuming timer
	}
    }
void ADC1_IRQHandler(){
	HAL_ADC_IRQHandler(&hadc); 
}

/******************Functions****************************/
// FUNCTION      : controlInit()
// DESCRIPTION   : This function initializes timer 1 channel 1 as output, ADC1 channel 6 with interrupt after end of regular conversion, LCD, and associated GPIOs
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void controlInit(void *data){
  
  // Enabling clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  //configuring timer 1 channel 1 as PWM output
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  tim1.Instance = TIM1;
  tim1.Init.Prescaler = 35;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = 65500;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&tim1);
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = 0;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
  //configuring ADC 1 channel 6	
  GPIO_InitStruct.Pin = (GPIO_PIN_0);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE; 
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.NbrOfDiscConversion = 0;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = 0;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc);				//initializing ADC
  ADC_ChannelConfTypeDef config;
  config.Channel = 6;
  config.Rank = 1;
  config.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc, &config);

  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 3);	
  HAL_NVIC_EnableIRQ(ADC1_IRQn);		
  
  ADC1->IER |= ADC_IER_EOCIE; //enabling end of regular conversion interrupt
  ADC1->CR |= ADC_CR_ADEN;    //enabling ADC
  
  HD44780_Init();	//initializing LCD	
}
