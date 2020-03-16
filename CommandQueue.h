#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

//function declerations
void addToQueue(uint32_t time, int16_t speed);
struct queue* extractFromQueue();

//structure for the queue
struct queue
{
  struct queue* next;   //pointer to next node in queue
  int16_t speed;
  uint32_t time;
};



