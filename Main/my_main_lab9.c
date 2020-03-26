/* my_main.c: main file for monitor */
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include "common.h"
#include "DC_CommandQueue.h" //header file for queue
#include "main.h"		  // This include will give us the CubeMX generated defines

//global variables 
extern uint8_t timerDone[2];    // flag that is set to 1 when the timer 1 is done.
extern uint16_t queueCounter[2]; // Counter that records the number of nodes in the command queue.
extern uint16_t dutyCycle;
extern uint32_t counter[2];

//function declarations
void DC(uint16_t channel, uint16_t dutyCycle, uint16_t direction);
void motorStop(uint16_t channel);

/* This function is called from the CubeMX generated main.c, after all
 * the HAL peripherals have been initialized. */
void my_init(void){
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
void my_main(void){
	struct queue* data;// used to point to the node in the queue that has the data that will be read.
	TaskingRun();  /* Run all registered tasks */
	my_Loop();
	WDTFeed();

	for(int i=0;i<2;i++){
		if(queueCounter[i] > 0 && timerDone[i] == 1){ //checking if queue is not empty and channel is not busy
			timerDone[i] = 0;      //setting flag for channel as busy
			data = extractFromQueue(i+1);   //extracting the first node's data from queue
			if(data->direction == 0){
				motorStop(i+1);				//stopping motor
			}else{
				DC(data->channel, data->dutyCycle, data->direction);
				counter[i] = data->time; //calculating the number of overflows needed	
			}	
    		}
  	}
}
