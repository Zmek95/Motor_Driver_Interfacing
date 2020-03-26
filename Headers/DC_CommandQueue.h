#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

//function declerations
void addToQueue(uint16_t channel, uint16_t dutyCycle, uint16_t direction, uint32_t time);
struct queue* extractFromQueue(uint16_t channel);

//structure for the queue
struct queue
{
  struct queue* next;   //pointer to next node in queue
  uint16_t dutyCycle;
  uint32_t time;
  uint16_t direction;
  uint16_t channel;
};



