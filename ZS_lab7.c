/* 		ZS_lab7.c
 *              Stepper motor controls Part II
 *		Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 *		Date: Feb 27, 2020
 */

#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "CommandQueue.h"

#define RST   GPIO_PIN_3
#define PS    GPIO_PIN_8
#define OE    GPIO_PIN_4
#define FR    GPIO_PIN_5

//function declarations
void stepperInit(void);
void PWMInit(void);
void gpioStepperInit(void);
void StepEnable(uint32_t enable);
void dac1init(void);
void runningTime(uint32_t time);



/*** global variables ***/
uint32_t counter = 0;	// counts the number of times the tim15 counter has to overflow
uint16_t remaining = 0; // used to start the CNT register for tim15 at a higher number when the remaining time is less than one period
uint16_t period = 0;	// time period of step pulses
uint16_t timerDone = 1; // flag that is set to 1 when timer 15 is not running.

TIM_HandleTypeDef tim15;

/***************************************************************************/
// FUNCTION      : StepperInit()
// DESCRIPTION   : This function will intialize the periphirals (GPIO, DAC1, Timer 15 channel 1 as PWM, enable interrupt at overflow) 
//                 needed for the stepper motor and intialize the pins for the stepper motor driver.
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void stepperInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  
  // Enabling clock for GPIOs
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  //configure GPIOC pins	
  GPIO_InitStruct.Pin = RST | PS | OE | FR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  //configure GPIOB pins	
  GPIO_InitStruct.Pin = GPIO_PIN_14;//PWM output to STEP pin
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	

  HAL_GPIO_WritePin(GPIOC,RST,0);//make sure driver is in reset mode

  //Initialize DAC
  dac1init();

  //Enable clock for timer 15
  __HAL_RCC_TIM15_CLK_ENABLE();
  //Configure timer 15
  tim15.Instance = TIM15;
  tim15.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/1000000 - 1;
  tim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim15.Init.Period = 0;
  tim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim15.Init.RepetitionCounter = 0;
	
  TIM15 -> CR1 |= (TIM_CR1_URS); //only counter over/under flow generates interrupt
  TIM15 -> DIER |= 0b1; //setting Update Interrupt Enable bit 
  TIM15 -> EGR |= 0b1; //setting Update Generation bit 
  HAL_TIM_Base_Init(&tim15);
	
  //Configure timer 15 for PWM generation
  TIM_OC_InitTypeDef  sConfig;
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse  = 0;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim15,&sConfig,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim15,TIM_CHANNEL_1);		
  //Enable Interrupts 
  HAL_NVIC_SetPriority(TIM15_IRQn,0,1); 
  NVIC_EnableIRQ(TIM15_IRQn);

  //Configure driver
  HAL_GPIO_WritePin(GPIOC,PS,1);
  HAL_GPIO_WritePin(GPIOC,OE,1);
  HAL_GPIO_WritePin(GPIOC,FR,0);
  HAL_GPIO_WritePin(GPIOC,RST,1);
}

// FUNCTION      : StepperEnable()
// DESCRIPTION   : This function will enable the stepper motor by enabling the OE pin on the stepper motor driver.
// PARAMETERS    : uint32_t enable : a '0' sets the OE pin high otherwise the OE pin is set low. The stepper motor is enabled
//                 when the OE pin is set low.
// RETURNS       : Nothing
void StepEnable(uint32_t enable)
{
  HAL_GPIO_WritePin(GPIOC,OE,enable ? 0 : 1);
}

// FUNCTION      : runningTime()
// DESCRIPTION   : This function will calculate the time that the stepper motor needs to operate for 
// PARAMETERS    : uint32_t time : represents time in seconds for the desired iteration time for the stepper motor to keep running
// RETURNS       : Nothing
void runningTime(uint32_t time)
{
  counter = (time * 1000000) / period;   //number of times the CNT register will need to overflow
  remaining = (time * 1000000) % period; //used to store the remaining time (less than one period)
}
/***************ISR***************/
// FUNCTION      : TIM15_IRQHandler()
// DESCRIPTION   : Interrupt for controlling the number of counts required for the TIM15 counter register.
//                 Used to get the correct timings for the stepper motor. It also raises a flag when timer is done
//                 to control the flow of the queue
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void TIM15_IRQHandler(void)
{
  if((counter == 0) && (remaining == 0))//This branch is entered when the timer is done
    {
      timerDone = 1;                  //flag used to control the flow of commands in the queue
      TIM15 -> CR1 &= ~TIM_CR1_CEN;
		
    } else if(counter == 0)// this branch is entered when the time remaining is less than 1 period
    {
      TIM15 -> CNT = (period + 1) - remaining;
      remaining = 0;
    }else
    {
      counter--;
    }

  TIM15 -> SR &= 0xfffe;// Resetting the Update Interrupt Flag (UIF) to 0
}
/*************Commands****************************/
//This command intiates the periphirals that are used by the stepper motor driver
ParserReturnVal_t CmdStepperinit(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  stepperInit();
  return CmdReturnOk;
}
ADD_CMD("stepinit",CmdStepperinit,"initializes stepper motor")

//This command enables the stepper motor by enabling the OE pin 
ParserReturnVal_t CmdStepEnable(int mode)
{
  uint32_t enable, rc;
  
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  rc = fetch_uint32_arg(&enable);
  if (rc)
    {
      printf("Please supply 1|0 to turn on or off the stepper motor\n");
      return CmdReturnBadParameter1;
    }
  if(!(enable == 0 || enable == 1))
    {
      printf("Please supply 1|0 to turn on or off the stepper motor\n");
      return CmdReturnBadParameter1;  
    }
  StepEnable(enable);
  return CmdReturnOk;
}
ADD_CMD("stepEnable",CmdStepEnable,"Enables stepper motor")

//this command accepts the user entered parameters of time(seconds) and speed(revolution/second)
//to move the stepper motor shaft at the user entered speed for the user entered time.
ParserReturnVal_t CmdstepSpeed(int mode)
{
  int rc = 0;
  uint32_t time = 0;	//time in seconds set by user for stepper motor to continue rotating
  int16_t speed = 0;	//angular velocity in rotations per second
	

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
	
  rc = fetch_uint32_arg(&time);
  if (rc)
    {
      printf("Please enter amount of time in seconds the stepper motor to work\n");
      return CmdReturnBadParameter1;
    }
  rc = fetch_int16_arg(&speed);
  if (rc)
    {
      printf("Please enter speed of rotation (rotation per second)\n");
      return CmdReturnBadParameter2;
    }
  if(speed < 0)//Checks direction of rotation
    {
      speed = -speed; //taking only the amplitude	
      HAL_GPIO_WritePin(GPIOC,FR,1);
    }
  else
    {
      HAL_GPIO_WritePin(GPIOC,FR,0);
    }
	
  if(time <= 0)
    {
      printf("Please input a postive integer for time in seconds\n");
      return CmdReturnBadParameter1;
    }
	
  if((speed > 12) || (speed < 0)) //avoiding invalid entries; stepping frequency should not go higher than 20kHz
    {
      printf("Invalid entry, speed can only be between 0 and 12.5 revolutions per second\n");
      return CmdReturnBadParameter2;
    }
  addToQueue(time, speed);
  return CmdReturnOk;
}
ADD_CMD("stepSpeed",CmdstepSpeed,"<time> <speed>          sets speed (rev/s) & time (s) for motor and starts timer")
