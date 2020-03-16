/*   lab 3 ADC 
     programmed by:
     Ziyad Mekhemer & Shamseddin Elmasri
     Jan 23, 2020
*/

#include <stdio.h>
#include <stdint.h>

#include "common.h"

ADC_HandleTypeDef hadc;

void adcInitialization(int mode)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};	
  // Enabling clock for ADC GPIOs
  __HAL_RCC_GPIOC_CLK_ENABLE();
  //configure GPIO pins as Analog	
  GPIO_InitStruct.Pin = (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  //configure PC13 as input
  GPIO_InitStruct.Pin = (GPIO_PIN_13);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
}
// function for ADC command which initializes ADC and pushbutton
ParserReturnVal_t CmdADCinit(int mode)
{
  HAL_StatusTypeDef rc;   
  if(mode != CMD_INTERACTIVE) return CmdReturnBadParameter1;
  adcInitialization(mode);
  rc= HAL_ADC_Init(&hadc);
  if (rc != HAL_OK)
    {
      printf("ADC1 initialization failed with rc=%u\n",rc);
    }
  else{
    printf("ADC and pushbutton initialized\n");
  }
  return CmdReturnOk; 
}
ADD_CMD("ADCinit",CmdADCinit,"initializes ADC and pushbutton")

uint32_t ADCRead(uint32_t channel)
{
  uint32_t val;
  HAL_StatusTypeDef rc;
  ADC_ChannelConfTypeDef config;
  /* Configure the selected channel */
  config.Channel = channel;
  config.Rank = 1;
  config.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  config.SingleDiff = ADC_SINGLE_ENDED;
  config.OffsetNumber = ADC_OFFSET_NONE;
  config.Offset = 0;
  rc = HAL_ADC_ConfigChannel(&hadc,&config);
  if(rc != HAL_OK) {
    printf(
	   "ADC channel configure failed with rc=%u\n",
	   (unsigned)rc);
    return 0;
  }
  /* Start the ADC peripheral */
  rc = HAL_ADC_Start(&hadc);
  if(rc != HAL_OK) {
    printf(
	   "ADC start failed with rc=%u\n",
	   (unsigned)rc);
    return 0;
  }
  
  /* Wait for end of conversion */
  rc = HAL_ADC_PollForConversion(&hadc, 100);
  if(rc != HAL_OK) {
    printf(
	   "ADC poll for conversion failed with "
	   "rc=%u\n",(unsigned)rc);
    return 0;
  }
  /* Read the ADC converted values */
  val = HAL_ADC_GetValue(&hadc);
  /* Stop the ADC peripheral */
  rc = HAL_ADC_Stop(&hadc);
  if(rc != HAL_OK) {
    printf("ADC stop failed with "
	   "rc=%u\n",(unsigned)rc);
    return 0;
  }
  return val;
}
ParserReturnVal_t CmdADC(int mode)
{
  uint32_t adcVal;
  uint32_t channel1, channel2;
  uint32_t rc;
  float voltage;
  int nOfStars; //used to store number of stars to create horizontal bars
  int pushb = 1; //used to store state of pushbutton
  if(mode != CMD_INTERACTIVE) return CmdReturnBadParameter1;
  //prompt user to choose first channel
  rc = fetch_uint32_arg(&channel1);
  if (rc) {
    printf("Must supply channel numbers\n");
    return CmdReturnBadParameter1;
  }
  if ((channel1 < 6) | (channel1 > 9))//choosing channels only from PortC
    {printf("channel number is out of scope, choose 6, 7, 8 or 9\n");
      return CmdReturnBadParameter1;
    }
  //prompt user to choose second channel
  rc = fetch_uint32_arg(&channel2);
  if (rc) {
    printf("Must supply channel2 number\n");
    return CmdReturnBadParameter1;
  }
  if ((channel2 < 6) | (channel2 > 9))//choosing channels only from PortC
    {printf("channel number is out of scope, choose 6, 7, 8 or 9\n");
      return CmdReturnBadParameter1;
    }
  while (pushb == 1) //checking state of pushbutton
    {//reading from channel1
      adcVal = ADCRead(channel1);
      printf("ADC %li value: %lu\n",channel1,adcVal);
      //converting value into voltage
      voltage = adcVal * 3.3 / 4095;
      printf("voltage1 = %.2fV\n", voltage);
      nOfStars = adcVal * 80 / 4095;
      //displaying vertical bars
      for (int i = 0; i <= nOfStars; i++)
	printf("*");
      printf("\n");
      //reading from channel2
      adcVal = ADCRead(channel2);
      printf("ADC %li value: %lu\n",channel2,adcVal);
      //converting value into voltage
      voltage = adcVal * 3.3 / 4095;
      printf("voltage2 = %.2fV\n", voltage);
      nOfStars = adcVal * 80 / 4095;
      //displaying vertical bars
      for (int i = 0; i <= nOfStars; i++)
	printf("*");
      
      HAL_Delay(400);
      //clearing lines
      for (int i=0;i<5;i++)
	{printf("\33[2K");
	 printf("\033[A");
	}
      printf("\33[2K");
      printf("\r");
      //reading pushbutton
      pushb = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
    }
  return CmdReturnOk;
}

ADD_CMD("ADC",CmdADC,"reads from ADC channel")

