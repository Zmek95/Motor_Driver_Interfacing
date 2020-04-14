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
uint16_t getADCValue(void);
void HD44780_Init(void);
void HD44780_GotoXY(unsigned char x, unsigned char y);
void HD44780_PutStr(char *str);
void HD44780_ClrScr(void);
void displayLCD(void);
//global variables
uint32_t counter = 1; 		//used in ISR to count number of overflows for tim1
char stringBuffer[16]= {0};     //buffer used to store different datatypes for LCD

/******************Functions****************************/
// FUNCTION      : PWMInit()
// DESCRIPTION   : Intializes the GPIO pins and timer 1 to be configured for PWM output 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void PWMInit(void) {

  // Enabling clock for GPIOs
  __HAL_RCC_GPIOA_CLK_ENABLE();
  //configure GPIO pins	
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //Enabling clock for TIM1
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
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

  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 2);//preemption 1, priority 2
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}
// FUNCTION      : adcInit()
// DESCRIPTION   : Intializes ADC1 channel 6 
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void adcInit(void){
	
  // Enabling clock for ADC GPIOs
  __HAL_RCC_GPIOC_CLK_ENABLE();
  //configure GPIO pins as Analog	
  GPIO_InitStruct.Pin = (GPIO_PIN_0);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  //Initialize ADC
  __HAL_RCC_ADC1_CLK_ENABLE();
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
  /* Configure the selected channel */
  config.Channel = 6;
  config.Rank = 1;
  config.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc, &config);
}

// FUNCTION      : getADCValue()
// DESCRIPTION   : Performs an analog to digital conversion and returns the converted value
// PARAMETERS    : Nothing
// RETURNS       : volatile uint16_t uhADCxConvertedValue - represents the raw value of the current output
uint16_t getADCValue(void){
  volatile uint16_t uhADCxConvertedValue = 0;
  HD44780_ClrScr(); 				      //clearing LCD screen
  if (HAL_ADC_Start(&hadc) != HAL_OK){ /* Start ADC */
    HD44780_PutStr("ADC START ERROR");
    return CmdReturnOk;
  }
  if (HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK){
    HD44780_PutStr("CONVERSION ERROR");
    return CmdReturnOk;
  }else{/* ADC conversion completed */
    uhADCxConvertedValue = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    return uhADCxConvertedValue;
  }
}
void displayLCD(void){
  if(counter%10 == 0){
    uint32_t feedbackValue = getADCValue() * 100 / 722 ;	     //getting feedback value from ADC
    sprintf(stringBuffer,"%lu prcnt setp",feedbackValue);
    HD44780_PutStr(stringBuffer);				     //displaying feedback value on LCD		
    HD44780_GotoXY(0,1);
    sprintf(stringBuffer,"@ %lu dutyC",TIM1->CCR1 * 100 / 65500);
    HD44780_PutStr(stringBuffer);				     //displaying feedback value on LCD		
  }
}
/*************Commands****************************/
ParserReturnVal_t CmdLoopInit(int mode) {

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;

  PWMInit();	//initialize timer 1 channel 1 as PWM
  adcInit();	//initialize ADC1 on PC0
  HD44780_Init();	//initialize LCD
  HD44780_PutStr("Lab 11 demo");//displaying feedback value on LCD		
  return CmdReturnOk;
}
ADD_CMD("LoopInit", CmdLoopInit, "		intializes PWM on timer 1 channel 1, ADC1 channel 6,  and LCD")

ParserReturnVal_t CmdSetPWM(int mode) {

  if (mode != CMD_INTERACTIVE) return CmdReturnOk;
  uint16_t dutyCycle;
	
  fetch_uint16_arg(&dutyCycle);		//inputting duty cycle
  TIM1->CR1 &= ~TIM_CR1_CEN;		//stopping timer
  TIM1->CR1 |= (TIM_CR1_URS); 		//only counter over/under flow generates interrupt
  TIM1->DIER |= 0b1; 			//setting Update Interrupt Enable bit 
  TIM1->EGR |= 0b1;			//setting Update Generation bit 
  TIM1->CCR1 = 655 * dutyCycle;   	//setting duty cycle
  TIM1->CR1 |= TIM_CR1_CEN;		//starting timer
	
  return CmdReturnOk;
}
ADD_CMD("SetPWM", CmdSetPWM, "		Sets duty cycle of PWM")

/***************************ISR****************************/

// FUNCTION      : TIM1_UP_TIM16_IRQHandler()
// DESCRIPTION   : This ISR is visited everytime TIM1 overflows.
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM1_UP_TIM16_IRQHandler(void) {
  TIM1->CR1 &= ~TIM_CR1_CEN; 			      //stopping timer	
  if(counter%10 == 0){
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);	      //indicater 
  }
  counter++;	      
  TIM1->CR1 |= TIM_CR1_CEN;	//resuming timer
  TIM1 -> SR &= 0xfffe;         // Resetting the Update Interrupt Flag (UIF) to 0
}
