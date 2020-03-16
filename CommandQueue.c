#include "CommandQueue.h"

struct queue* front = NULL; //pointer to node ready for execution
struct queue* rear = NULL;  //pointer to last node added to queue

//global variables
uint16_t queueCounter = 0; //counts number of elements in the queue

/***************************************************************************/
// FUNCTION      : addToQueue()
// DESCRIPTION   : This function adds a new node to the queue. 
//		   This node contains the time and speed entered by the user along with a pointer to the next node
// PARAMETERS    : uint32_t time: holds time in seconds
// 		   int16_t speed: holds speed in rotations per seconds
// RETURNS       : Nothing
void addToQueue(uint32_t time, int16_t speed)
{
  struct queue* newNode = NULL;		//used for pointing to new node
  newNode = (struct queue*)malloc(sizeof(struct queue));

  if (newNode != NULL)	//checking if enough memory is available
    {	//creating a new node
      newNode->speed = speed;	
      newNode->time = time;
    }
  else
    {
      printf("No sufficient memory!\n");
      return;
    }
  if (rear != NULL)	//checking if the list is empty
    {
      rear->next = newNode; //linking newNode with queue
    }
  else
    {
      front = newNode; //front pointer is pointing to first node in queue
    }
  rear = newNode;   	     //placing new node as head
  queueCounter++;		     //increment number of elements in queue
  rear->next = NULL;	    //rear's next pointer points to NULL
}
/***************************************************************************/
// FUNCTION      : extractFromQueue()
// DESCRIPTION   : This function extracts the first node in the queue
//		   This node contains the time and speed entered by the user along with a pointer to the next node
// PARAMETERS    : Nothing
// RETURNS       : struct queue* data: this pointer points to the node that contains the information needed (time and speed)
struct queue* extractFromQueue()
{
  struct queue* pointer = NULL; //temporary pointer
  struct queue* data = NULL;    //pointer will be used to point to first node in queue
	
  data = front; 	//copying address of node containing data
  pointer = front;//pointing to front of queue
  //checking if queue has only one node left
  if(front == rear){	
    front = NULL;
    rear = NULL;
    printf("queue is now empty\n");
  }else{
    front = front->next;//moving one node back from front of queue
  }

	
  free(pointer); //freeing memory allocated
  queueCounter--;//decrementing number of elements in queue
  return data;
}










