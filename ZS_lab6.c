/* 		ZS_lab6.c
 *      Stepper motor controls Part I
 *		Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 */
#include <stdio.h>
#include <stdint.h>
#include "common.h"

//function declarations
void timerInit(void);
void timerStepperDelay(uint32_t delay);
void dac1init(void);

//global variables
TIM_HandleTypeDef tim17;
uint32_t counter = 0;// counts the number of times the tim17 counter has to overflow
uint16_t remaining = 0;// used to start the CNT register for tim17 at a higher number when the remaining time is less than 65536 microseconds
uint32_t userDelay = 0;	//used to store value of delay input from user
uint32_t stopTimerCounter = 0;// counts the number of times the timer has to execute, this is related to the number of steps the motor has to take.
float variableDelay = 0;//used to vary delay for the trapaziodal function
int32_t userSteps = 0;//used to store the number of steps for the stepper motor

#define RST   GPIO_PIN_3
#define PS    GPIO_PIN_8
#define OE    GPIO_PIN_4
#define STEP  GPIO_PIN_14
#define FR    GPIO_PIN_5
/***************************************************************************/
// FUNCTION      : StepperInit()
// DESCRIPTION   : This function will intialize the periphirals needed for the stepper motor driver
// PARAMETERS    : Nothing
// RETURNS       : Nothing
void StepperInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  
  // Enabling clock for GPIOs
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  //configure GPIO pins	
  GPIO_InitStruct.Pin = RST | PS | OE | FR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = STEP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  HAL_GPIO_WritePin(GPIOC,RST,0);//make sure driver is in reset mode
  //Initialize DAC
  dac1init();
  //Intialize Timer
  timerInit();
  //Configure driver
  HAL_GPIO_WritePin(GPIOC,PS,1);
  HAL_GPIO_WritePin(GPIOC,OE,1);
  HAL_GPIO_WritePin(GPIOB,STEP,0);
  HAL_GPIO_WritePin(GPIOC,FR,0);
  HAL_GPIO_WritePin(GPIOC,RST,1);
}

// FUNCTION: StepEnable()
// DESCRIPTION:This function will enable the stepper motor by enabling the OE pin on the stepper motor driver.
// PARAMETERS: enable - a 0 sets the OE pin high otherwise the OE pin is set low. The stepper motor is enabled when the OE pin is set low.
// RETURNS: Nothing
void StepEnable(uint32_t enable)
{
  HAL_GPIO_WritePin(GPIOC,OE,enable ? 0 : 1);
}

// FUNCTION      : Step()
// DESCRIPTION   : This function will take a number of steps and a delay to make the stepper motor shaft spin. Global variable "stopTimerCounter" will
//		   hold the number of iterations (steps*2).
// PARAMETERS    :
//   steps - the number of steps the stepper motor takes, the stepper motor used in our lab takes 1600 steps 
//	     for a full revolution.
//   delay - the time delay between steps in microseconds 
// RETURNS       :Nothing
void Step(int32_t steps, uint32_t delay)
{  
  userDelay = delay;
  if(steps<0)//Checks direction the stepper motor will rotate
    {
      steps = -steps; 	
      HAL_GPIO_WritePin(GPIOC,FR,1);
    }
  else
    {
      HAL_GPIO_WritePin(GPIOC,FR,0);
    }
  userSteps = steps;
  stopTimerCounter = steps*2;//The timer needs to be twice the number of steps to toggle the STEP pin on and off.
  HAL_GPIO_WritePin(GPIOB,STEP,1);
  //interrupt driven delay starts here
  timerStepperDelay(delay); 
}

//Function name: timerInit()
//Description: Initializes timer17 to have a 1 microsecond tick, and enables the overflow interrupt
//Parameters: void
//Returns: void
void timerInit(void)
{
  __HAL_RCC_TIM17_CLK_ENABLE();
  tim17.Instance = TIM17;
  tim17.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/1000000 - 1;
  tim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim17.Init.Period = 0xffff;
  tim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim17.Init.RepetitionCounter = 0;
  TIM17 -> CR1 |= (TIM_CR1_URS); //only counter over/under flow generates interrupt
  TIM17 -> DIER |= 0b1; //setting Update Interrupt Enable bit
  TIM17 -> EGR |= 0b1; //setting Update Generation bit
  HAL_TIM_Base_Init(&tim17);
	
  //Enable Interrupts
  HAL_NVIC_SetPriority(TIM17_IRQn,0,1);
  NVIC_EnableIRQ(TIM17_IRQn);
}
//Function name: timerStepperDelay(uint32_t delay)
//Description: the time delay that is used to control the time intervals between STEP toggles of the stepper motor
//Parameters: delay - delay time in microseconds
//Returns: void
void timerStepperDelay(uint32_t delay)
{
  counter = delay/65536;//number of times the CNT register will need to overflow
  remaining = delay%65536;//used to store the remaining time under 65536
	
  HAL_TIM_Base_Start(&tim17); //starts timer
	
  if(counter == 0)// checks if the delay is less than 65536 microseconds
    {
      TIM17 -> CNT = 65536 - remaining;
      remaining = 0;
    }
  else
    {
      TIM17 -> CNT = 65535;
    }
}
/*************************Commands***********************/
//This command intiates the periphirals that are used by the stepper motor driver
ParserReturnVal_t CmdStepperinit(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

  StepperInit();
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
ADD_CMD("stepEnable",CmdStepEnable,"initializes stepper motor")

//This command takes revolutions/second and rotation angle from user to move the stepper motor shaft
ParserReturnVal_t CmdStep(int mode)
{
  int32_t steps; //number of microsteps
  float speed; //revolutions per second
  uint32_t timerCounts; //in microseconds
  int16_t angle; //in degrees
  int rc;
 
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

  rc = fetch_int16_arg(&angle); 
  if (rc)
    {
      printf("Please supply the rotation of the shaft in degrees for the stepper motor\n");
      return CmdReturnBadParameter1;
    }
  rc = fetch_float_arg(&speed); 
  if (rc)
    {
      printf("Please supply stepper motor speed (revolutions per second)\n");
      return CmdReturnBadParameter2;
    }
  if((speed > 12.5) || (speed < 0)) //avoiding invalid entries; stepping frequency should not go higher than 20kHz
    {
      printf("Invalid entry, speed can only be between 0 and 12.5 revolutions per second\n");
      return CmdReturnBadParameter3;
    }
  timerCounts = 500000 / (1600 * speed); //calculating number of timer ticks needed for each pulse-width
  steps = angle*1600/360;	//calculting number of steps required to rotate the specified angle
  Step(steps,timerCounts);	//generating step
  return CmdReturnOk;
}
ADD_CMD("step",CmdStep,"<step> <delay> Make stepper motor step")

/*************************************Interrupt***************************************************/
//Function name: TIM17_IRQHandler
//Description: Interrupt for controlling the number of counts required for the TIM17 counter register.
//             Used to get the correct timings for timerStepperDelay, the STEP pin is toggled when the 
// 	       timer is finished, the timer will then execute again for the number of steps*2 the stepper
//             motor needs to take. 
//Parameters: void
//Returns: void
void TIM17_IRQHandler(void)
{
  if((counter == 0) && (remaining == 0))//This branch is entered when the timer is done
    {
      HAL_GPIO_TogglePin(GPIOB, STEP);
      if ((userSteps-stopTimerCounter/2) > (userSteps*14/15)) //motor deceleration
	{
	  variableDelay += (userSteps-stopTimerCounter/2)/2000;
	  HAL_TIM_Base_Stop(&tim17);
	  timerStepperDelay((uint32_t)variableDelay);
	}	
      else if((userSteps-stopTimerCounter/2) < (userSteps/15))	//motor acceleration
	{
	  variableDelay = userDelay;    //unfortunetly we didn't have enough time to code this part  
	}
      else
	{
	  variableDelay = userDelay;	
	}
      if(stopTimerCounter == 0)
	{
	  HAL_TIM_Base_Stop(&tim17);
	  HAL_GPIO_WritePin(GPIOB, STEP, 0);
	}
      else
	{
	  stopTimerCounter--;
	  HAL_TIM_Base_Stop(&tim17);
	  timerStepperDelay((uint32_t)variableDelay);//call timer again to start the timer after it is done
	} 
    }
  else
    {
      if(counter == 0)// this branch is entered when the time remaining is less than 65536
	{
	  TIM17 -> CNT = 65536 - remaining;
	  remaining = 0;
	}
      else
	{
	  counter--;      
	}
    } 
  TIM17 -> SR &= 0xfffe;// Resetting the Update Interrupt Flag (UIF) to 0                    
}
