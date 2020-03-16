/* my_main.c: main file for monitor */
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "common.h"
#include "CommandQueue.h" //header file for queue

/* This include will give us the CubeMX generated defines */
#include "main.h"

//global variables 
extern uint16_t timerDone;    // flag that is set to 1 when the timer 15 is done.
extern uint16_t queueCounter; // Counter that records the number of nodes in the command queue.
extern uint16_t period;       // time period of step pulses.

//function declarations
void runningTime(uint32_t time);

/* This function is called from the CubeMX generated main.c, after all
 * the HAL peripherals have been initialized. */
void my_init(void)
{
  /* Initialize the terminal system */
  TerminalInit();

  /* Print out reset source */
  WDTCheckReset();
  
  /* Initialize the task system */
  TaskingInit();

  my_Init();

}

/* This function is called from inside the CubeMX generated main.c,
 * inside the while(1) loop. */
void my_main(void)
{
  struct queue* data;// used to point to the node in the queue that has the data that will be read.
  TaskingRun();  /* Run all registered tasks */
  my_Loop();
  WDTFeed();
  if(queueCounter > 0 && timerDone == 1) //checking if queue is not empty and timer is not busy
    {	
      timerDone = 0;      //setting flag for timer as busy
      data = extractFromQueue();   //extracting the first node's data from queue
      period = 1000000 / (1600 * data->speed); //calculating number of timer ticks needed for each pulse-width
      runningTime(data->time); //calculates the number of overflows needed		
      TIM15 -> CR1 &= ~TIM_CR1_CEN;//stopping timer
      TIM15 -> ARR = period;	//setting period
      TIM15 -> CCR1 = period/2;//setting pulse width
      TIM15 -> CR1 |= TIM_CR1_CEN;//starting timer
    }
}

ParserReturnVal_t CmdLed(int mode)
{
  uint32_t val,rc;
  
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

  rc = fetch_uint32_arg(&val);
  if(rc) {
    printf("Please supply 1 | 0 to turn on or off the LED\n");
    return CmdReturnBadParameter1;
  }

  val = val ? 1 : 0;
#if defined(LD2_Pin)
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,val);
#elif defined(LD3_Pin)
  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,val);
#endif
  
  return CmdReturnOk;
}

ADD_CMD("led",CmdLed,"0 | 1           Control LED")

