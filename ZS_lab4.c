/* ZS_lab4.c
 *  Digital to Analog Converter
 *  Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *  Date: Feb 03, 2020
 */

#include <stdio.h>
#include <stdint.h>

#include "common.h"

HAL_StatusTypeDef dacSetValue(float value);
void dac1init(void);

static DAC_HandleTypeDef hdac;
//static TIM_HandleTypeDef tim6;
//static DMA_HandleTypeDef dma1;

void dac1init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  DAC_ChannelConfTypeDef DacConfig;
  HAL_StatusTypeDef rc;

  /*Initializing GPIO*/
  // Enabling clock for GPIOA
  __HAL_RCC_GPIOA_CLK_ENABLE();
  //configuring GPIOA pin 4 as Analog
  GPIO_InitStruct.Pin = (GPIO_PIN_4);
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Initializing DAC*/
  //Enabling DAC1 clock
  __HAL_RCC_DAC1_CLK_ENABLE();

  hdac.Instance = DAC1;
  rc = HAL_DAC_Init(&hdac);
  if(rc != HAL_OK)
    {
      printf("Unable to initialize DAC, rc = %d\n",rc);
      return;
    }


  DacConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  DacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  rc = HAL_DAC_ConfigChannel(&hdac, &DacConfig, DAC_CHANNEL_1);
  if(rc != HAL_OK)
    {
      printf("Unable to configure DAC Channel1, rc = %d\n", rc);
      return;
    }
  /*Enabling DAC1 Output*/
  __HAL_DAC_ENABLE(&hdac,DAC_CHANNEL_1);
  //Setting initial value
  dacSetValue(0);

}

HAL_StatusTypeDef dacSetValue(float value)
{
  
  HAL_StatusTypeDef rc;
  int DACvalue;

  DACvalue =(int) ((value*4095.0) / 3.3);

  rc = HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, DACvalue);
  if(rc != HAL_OK)
    {
      printf("Unable to set value on DAC channel1, rc = %d\n", rc);
      return rc;
    }
  return HAL_OK;
}

ParserReturnVal_t CmdDAC1init(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
 
  dac1init();
  return CmdReturnOk;
}

ADD_CMD("DAC1init",CmdDAC1init,"Initializes DAC1")

ParserReturnVal_t CmdDAC1Val(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  int rc;
  float value;
  rc =fetch_float_arg(&value);
  if(rc != HAL_OK)
    {
      printf("You need to input a value, rc = %d\n",rc);
      return CmdReturnBadParameter1;
    }
  dacSetValue(value);
  return CmdReturnOk;
}

ADD_CMD("DAC1Val",CmdDAC1Val,"Sets value on DAC1")




//Arbitirary waveform generator function
//I referenced this website :https://vivonomicon.com/2019/07/05/bare-metal-stm32-programming-part-9-dma-megamix/
//heavily to help me in the dma initilazation.
//I used HAL functions for intialization at first but the HAL functions kept reseting some my direct register writes 
ParserReturnVal_t CmdDAC1ArbWave(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

  // Square wave initilazation
  const size_t SAMPLES = 32;
  const uint16_t waveform[32] = {4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  // Configure Perhiperals
  RCC->AHBENR   |= ( RCC_AHBENR_GPIOAEN |
                     RCC_AHBENR_DMA1EN );
  RCC->APB1ENR  |= ( RCC_APB1ENR_DAC1EN |
                     RCC_APB1ENR_TIM6EN );
  RCC->APB2ENR  |= RCC_APB2ENR_SYSCFGEN;

  //Timer 6 to give us a sample sequence at the some sample rate 
  TIM6->PSC  =  ( 0x000B );
  TIM6->ARR  =  60000;
  // Enable trigger output on timer update events.
  TIM6->CR2 &= ~( TIM_CR2_MMS );

  //DMA Initilazation
  DMA1_Channel3->CCR &= ~( DMA_CCR_MEM2MEM |  //Disable bits in register
			   DMA_CCR_PL |
			   DMA_CCR_MSIZE_Msk |
			   DMA_CCR_PSIZE_Msk |
			   DMA_CCR_PINC |
			   DMA_CCR_EN );
  DMA1_Channel3->CCR |=  ( ( 0x2 << DMA_CCR_PL_Pos ) | //Enable bits in register
			   ( 0x1 << DMA_CCR_MSIZE_Pos ) |
			   ( 0x1 << DMA_CCR_PSIZE_Pos ) |
			   DMA_CCR_MINC |
			   DMA_CCR_CIRC |
			   DMA_CCR_DIR );
 
  //CMAR contains starting address for the data that is in memory
  //CPAR contains address in the peripheral for the data from memory to be sent
  DMA1_Channel3->CMAR =(uint32_t) &waveform;
  DMA1_Channel3->CPAR =(uint32_t) &(DAC1->DHR12R1);
  DMA1_Channel3->CNDTR = (uint16_t) SAMPLES;// ********** set number of samples for the dma to look at. ARR enabled?
  DMA1_Channel3->CCR |= (DMA_CCR_EN);
 

  //DAC Setup
  DAC->CR &= ~( DAC_CR_TSEL1);// set register bits to 0 to select TIM6 as the trigger for DAC1
  DAC->CR |= (DAC_CR_DMAEN1);//Enable DAC DMA requests for channel 1
  DAC->CR |= (DAC_CR_EN1);//Enable DAC channel 1
  printf("Printing in 1 seconds!\n");
  HAL_Delay(1000);

  DAC->CR |= (DAC_CR_TEN1);//Enable conversions
  
  return CmdReturnOk;
}

ADD_CMD("DAC1ArbWave",CmdDAC1ArbWave,"Generates an arbitirary waveform of sine/triangle/square")
