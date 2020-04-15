/* 	ZS_lab11.c: Closed Loop Control
 *	Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *  	Date: Apr 12, 2020
 */
#include <stdio.h>
#include <stdint.h>
#include "common.h"

TIM_HandleTypeDef tim1;
TIM_OC_InitTypeDef  sConfig;
GPIO_InitTypeDef  GPIO_InitStruct = { 0 };
ADC_HandleTypeDef hadc;
//prototypes
void PWMInit(void);
void adcInit(void);
void HD44780_Init(void);
void HD44780_GotoXY(unsigned char x, unsigned char y);
void HD44780_PutStr(char *str);
void HD44780_ClrScr(void);
//global variables
char stringBuffer[16]= {0};     //buffer used to store different datatypes for LCD
volatile uint16_t controlMeasuredSpeed = 0;
uint16_t desiredSpeed = 0;
int Kp = 6;			//proportional constant
int32_t errorValue;	// to be put back as local after debugging complete
int32_t testNewValue;   //for debugging purpose only
/******************Functions****************************/
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
 // tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  tim1.Init.Prescaler = 35;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = 65500;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&tim1);
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = 10;
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
  HAL_ADC_Init(&hadc);
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
  
  ADC1->IER |= 0x4; //enable end of regular conversion interrupt setting EOCIE bit high
  ADC1->CR |= 1;  //enabling ADC
  
  //initializing LCD
  HD44780_Init();	
}

/*******************Tasks**************************/
void controlTask(void *data){
	
  static int LCDDelay = 0;
  uint16_t outputPercentage;

    HAL_ADC_Start_IT(&hadc);  

  if(LCDDelay % 25 == 0){
    printf("MeasuredSpeed = %u\terror = %ld\tnewValue = %ld\n ", controlMeasuredSpeed, errorValue/Kp, testNewValue);
    outputPercentage = controlMeasuredSpeed * 100 / desiredSpeed ;//getting feedback value from ADC
    HD44780_GotoXY(0,0);
    sprintf(stringBuffer,"%03d prcnt stp",outputPercentage);
    HD44780_PutStr(stringBuffer);      	                
    HD44780_GotoXY(0,1);
    sprintf(stringBuffer,"@ %03lu dutyCyc",TIM1->CCR1 * 100 / 65500);
    HD44780_PutStr(stringBuffer);			
  }
  LCDDelay++;
  
  //printf("TIM1->CCR1 = %lu\n", TIM1->CCR1);	
}
ADD_TASK(controlTask, controlInit, NULL, 20, "DC motor control task")

/*************Commands****************************/

ParserReturnVal_t CmdSetInput(int mode) {

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  fetch_uint16_arg(&desiredSpeed);		//inputting desired speed
  TIM1->CR1 &= ~TIM_CR1_CEN;    //stopping timer	
  TIM1->CCR1 = desiredSpeed * 65500 / 1738;
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
  HAL_ADC_Start_IT(&hadc);	//start ADC
  //printf("TIM1->CCR1 = %lu\n", TIM1->CCR1);
  return CmdReturnOk;
}
ADD_CMD("SetInput", CmdSetInput, "		Sets desired speed 0 - 1738")

/***************************ISR****************************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
    {   
	
	int32_t newValue;
        if(hadc1->Instance == ADC1){
  	  controlMeasuredSpeed = HAL_ADC_GetValue(&hadc);
	  
	  //PID controller code
	  if(controlMeasuredSpeed > desiredSpeed){
	    errorValue = controlMeasuredSpeed - desiredSpeed;
	  }else{
	    errorValue = desiredSpeed - controlMeasuredSpeed;
	  }

	  errorValue = errorValue * Kp;

	  if(desiredSpeed > controlMeasuredSpeed){
	    newValue = errorValue + desiredSpeed;
	  }else{
	    newValue = desiredSpeed - errorValue;
	  }
	  //setting limits for the new value
	  if(newValue > 1738){
	    newValue = 1738;
	  }else if(newValue < 0){
	    newValue = 0;
	  }
	  testNewValue = newValue;
	  if(errorValue != 0){
	    TIM1->CR1 &= ~TIM_CR1_CEN;  //stopping timer	
	    TIM1->CCR1 =  newValue * 65500 / 1738;
	    TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
	  }
	}
    }
void ADC1_IRQHandler(){
	HAL_ADC_IRQHandler(&hadc); 
}
