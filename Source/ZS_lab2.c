/*   lab 2 GPIO 
     programmed by:
Ziyad Mekhemer & Shamseddin Elmasri
Jan 22, 2020
*/

#include <stdio.h>
#include <stdint.h>

#include "common.h"



// function for gpioinit command which initializes Port A as output
ParserReturnVal_t Cmdgpioinit(int mode)
{
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  //initializing and seting Port A pins as outputs
  GPIO_InitTypeDef  GPIO_InitStruct;	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = (GPIO_PIN_0 |GPIO_PIN_1 |  GPIO_PIN_4 | GPIO_PIN_5 |
			       GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
			       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
			       GPIO_PIN_14 | GPIO_PIN_15);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	printf("PortA pins initialized\n");
	return CmdReturnOk;
}
ADD_CMD("gpioinit",Cmdgpioinit,"initializes port A pins except pins 2 and 3")

// function for GPIO command which controls the state of port A pins
ParserReturnVal_t CmdGPIO(int mode)
{
  uint32_t pinNumber,rc, state;
  uint32_t pinShifter = 0x01;
  
  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
  rc = fetch_uint32_arg(&pinNumber);
  if(rc)
    {
      printf("Please enter pin number to be controlled\n");
      return CmdReturnBadParameter1;
    }
  
  
  rc = fetch_uint32_arg(&state);
  
  if(rc)
    {
      printf("select 1 to turn on pin, or 0 to turn it off\n");
      return CmdReturnBadParameter1;
    }
  
  //Pins 2 and 3 are reserved for usart2 for the STM F303 board
  if(pinNumber == 2 || pinNumber == 3 || pinNumber > 15 || pinNumber < 0){
    printf("Please enter a valid GPIO port between 0-15 except pins 2 & 3\n");
    return CmdReturnOk;
  }
  
  
  GPIOA -> BSRR |= (pinShifter<<(pinNumber+(!(state))*16));	  
  
  return CmdReturnOk;
}

ADD_CMD("GPIO",CmdGPIO,"controlling Port A pins")

//Lab 1 command (names) which displays our first names based on users entry

ParserReturnVal_t CmdNames(int mode)
{
  uint32_t val,rc;
  
  if(mode != CMD_INTERACTIVE) return CmdReturnOk; //what is this for?
  
  rc = fetch_uint32_arg(&val);
  if(rc)
    {
      printf("Please supply 1 to display Shamseddin, or 0 for Ziyad\n");
      return CmdReturnBadParameter1;
    }
  
  if (val == 0)
    printf("Ziyad\n");
  else if (val == 1)
    printf("Shamseddin\n");
  else 
    printf("Invalid entry\n");
  
  return CmdReturnOk;
}

ADD_CMD("names",CmdNames,"enter 0 to display Ziyad, or 1 for Shamseddin")

