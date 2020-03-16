/* file name: ZS_lab5.c
 * Program name: Timer
 * Programmed by: Shamseddin Elmasri & Ziyad Mekhemer
 * Date: Feb 13, 2020
 */
#include <stdio.h>
#include <main.h>
#include "common.h"
/****************************************************/
//Global variables
TIM_HandleTypeDef tim17;
extern int TimerDoneFlag;
extern int timerStage;
uint32_t counter = 0;
uint16_t remaining = 0;
uint32_t multiTimerDelay[3] = {0,0,0};
/****************************************************/
//Function name: timerInit()
//Description: Initializes timer17 to have a 1 microsecond period
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
/****************************************************/
//Function name: timerDelay(uint32_t delayVal)
//Description: Starts the delay in microseconds 
//Parameters: delayVal (the time to be delayed in microseconds)
//Returns: void
void timerDelay(uint32_t delay)
{
  
  counter = delay/65536;//number of times the CNT register will need to overflow
  remaining = delay%65536;//used to start the CNT register at a higher number for the remaining time under 65536 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  
  HAL_TIM_Base_Start(&tim17);

  //trigger interrupt immediatly
  TIM17 -> CR1 &= ~(TIM_CR1_URS); 
  TIM17 -> EGR |= TIM_EGR_UG;//Enabling this bit allows the interrupt to trigger instantaneously
  TIM17 -> CR1 |= (TIM_CR1_URS);

  //TIM17 -> CNT = 65535;//We used this orginally to trigger the interrupt immediatly but it takes 1 microsecond to overflow
 
}
/****************************************************/
//Function name: timerDelay(uint32_t delayVal)
//Description: Interrupt for controlling the number of counts required for the TIM17 counter register.
//             Used to get the correct timings for timerDelay 
//Parameters: void
//Returns: void
void TIM17_IRQHandler(void)
{


  if((counter == 0) && (remaining == 0))//This branch is entered when the timer is done
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
      HAL_TIM_Base_Stop(&tim17);
      TimerDoneFlag = 1;
    } else if(counter == 0)// this branch is entered when the time remaining is less than 65536
    {
      TIM17 -> CNT = 65536 - remaining;
      remaining = 0;
    }else
    {
      counter--;
    }
  TIM17 -> SR &= 0xfffe;// Resetting the Update Interrupt Flag (UIF) to 0
                     
}
/****************************************************/
//Function name: multiTimer()
//Description: This function generates two delays after the first delay that is triggered by CmdmultiTimer. 
//             The delays only trigger after the completion of the previous delay, the "stage" the timer is
//             in is also recorded.
//Parameters: void
//Returns: void
void multiTimer(void){

  if (timerStage >= 1){
    timerStage++;
  }
  
  if(timerStage == 2){
    timerDelay(multiTimerDelay[1]);
  }else if(timerStage == 3){
    timerDelay(multiTimerDelay[2]);
  }else if (timerStage > 3){
    timerStage = 0; 
  }
  
  
}
/****************************************************/
//Function name: TimerDone()
//Description: Checks if the timer is done and calls the multiTimer function. This function
//             is placed in the while(1) loop in my_main_lab5.c 
//Parameters: void
//Returns: void
void TimerDone(void){

  if (TimerDoneFlag == 1){

    printf("Timer is done!\n");

    TimerDoneFlag = 0;
  
    multiTimer();
   
  }

}
/****************************************************/
//This command initializes and starts timer17
ParserReturnVal_t CmdtimerInit(int mode)
{
  timerInit();
  return CmdReturnOk;
}
ADD_CMD("timerInit",CmdtimerInit," Initializes and starts timer17")
/****************************************************/
//This command sets a delay using timer17
ParserReturnVal_t CmdtimerDelay(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  uint32_t rc;
  uint32_t delay = 0;
 
  rc = fetch_uint32_arg(&delay);
  if(rc) {
    printf("Enter value for delay\n");
    return CmdReturnBadParameter1;
  }
  
  timerDelay(delay);
  return CmdReturnOk;
}
ADD_CMD("timerDelay",CmdtimerDelay," sets a delay using timer17")
/****************************************************/
//This command checks the stage the multiTimer is in
ParserReturnVal_t CmdwhatStage(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

  if(timerStage == 1){

    printf("Melee\n");
    
  }else if(timerStage == 2){

    printf("Ranged\n");

  }else if(timerStage == 3){

    printf("Magic\n");
    
  }else{

    printf("MultiTimer is not active!\n");
  }
  
  return CmdReturnOk;
}
ADD_CMD("whatStage",CmdwhatStage," checks the stage the multiTimer is in")

/*********************************************************/
//This command takes three user inputs to generate three seperate timer blocks, where each block is a different stage that
//can be checked with whatStage command
ParserReturnVal_t CmdmultiTimer(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  uint32_t rc;
  

  rc = fetch_uint32_arg(&multiTimerDelay[0]);
  if(rc) {
    printf("Enter a positive integer for the delay\n");
    return CmdReturnBadParameter1;
  }

  rc = fetch_uint32_arg(&multiTimerDelay[1]);
  if(rc) {
    printf("Enter a positive integer for the delay\n");
    return CmdReturnBadParameter2;
  }

  rc = fetch_uint32_arg(&multiTimerDelay[2]);
  if(rc) {
    printf("Enter a positive integer for the delay\n");
    return CmdReturnBadParameter3;
  }

  timerDelay(multiTimerDelay[0]);
  timerStage++;

  return CmdReturnOk;

}
ADD_CMD("multiTimer",CmdmultiTimer," Starts the multi-stage timer which takes three positive integer delays")

