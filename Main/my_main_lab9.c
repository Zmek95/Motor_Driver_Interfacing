/* my_main.c: main file for monitor */
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include "common.h"
//#include "DC_CommandQueue.h" //header file for queue
#include "main.h"		  // This include will give us the CubeMX generated defines

void checkQueue(void);

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

	TaskingRun();  /* Run all registered tasks */
	my_Loop();
	WDTFeed();
	checkQueue(); //checking for ready commands in queue
}
