#include "DC_CommandQueue.h"

struct queue* front1 = NULL; //pointer to node ready for execution for channel 1
struct queue* rear1 = NULL;  //pointer to last node added to queue for channel 1
struct queue* front2 = NULL; //pointer to node ready for execution for channel 2
struct queue* rear2 = NULL;  //pointer to last node added to queue for channel 2
//global variables
uint16_t queueCounter[2] = {0,0}; //counts number of elements in the queue

/***************************************************************************/
// FUNCTION      : addToQueue()
// DESCRIPTION   : This function adds a new node to the queue. 
//		   This node contains the time, dutyCycle, channel, and direction entered by the user along with a pointer to the next node
// PARAMETERS    : uint32_t time: holds time in milliseconds
// 		   		   uint16_t dutyCycle: holds duty cycle of PWM
//				   uint16_t direction: holds the direction of rotation
//				   uint16_t channel: holds the channel number chosen
// RETURNS       : Nothing
void addToQueue(uint16_t channel, uint16_t dutyCycle, uint16_t direction, uint32_t time){
	struct queue* newNode = NULL;		//used for pointing to new node
	newNode = (struct queue*)malloc(sizeof(struct queue));

	if (newNode != NULL){	//checking if enough memory is available
		//creating a new node
		newNode->dutyCycle = dutyCycle;	
		newNode->time = time;
		newNode->direction = direction;
		newNode->channel = channel;
	}
	else{
		printf("No sufficient memory!\n");
		return;
	}
	if(channel == 1){	
		if (rear1 != NULL){	//checking if the list is empty
			rear1->next = newNode; //linking newNode with queue
		}
		else{
			front1 = newNode; //front pointer is pointing to first node in queue
		}
		rear1 = newNode;   	     //placing new node as head
		rear1->next = NULL;	    //rear's next pointer points to NULL
	}
	else{
		if (rear2 != NULL){	//checking if the list is empty
			rear2->next = newNode; //linking newNode with queue
		}
		else{
			front2 = newNode; //front pointer is pointing to first node in queue
		}
		rear2 = newNode;   	     //placing new node as head
		rear2->next = NULL;	    //rear's next pointer points to NULL
	}
	queueCounter[channel-1]++;		     //increment number of elements in queue	
}
/***************************************************************************/
// FUNCTION      : extractFromQueue()
// DESCRIPTION   : This function extracts the first node in the queue
//		   This node contains the channel, direction, dutyCycle and time entered by the user along with a pointer to the next node
// PARAMETERS    : Nothing
// RETURNS       : struct queue* data: this pointer points to the node that contains the information needed (time and speed)
struct queue* extractFromQueue(uint16_t channel){
	
	struct queue* pointer = NULL; //temporary pointer
	struct queue* data = NULL;    //pointer will be used to point to first node in queue
	
	if(channel == 1){
		data = front1; 	//copying address of node containing data
		pointer = front1;//pointing to front of queue1
		//checking if queue1 has only one node left
		if(front1 == rear1){	
			front1 = NULL;
			rear1 = NULL;
			printf("queue1 is now empty\n");
		}else{
			front1 = front1->next;//moving one node back from front of queue1
		}
	}else{
		data = front2; 	//copying address of node containing data
		pointer = front2;//pointing to front of queue2
		//checking if queue2 has only one node left
		if(front2 == rear2){	
			front2 = NULL;
			rear2 = NULL;
			printf("queue2 is now empty\n");
		}else{
			front2 = front2->next;//moving one node back from front of queue2
		}
	}	
	free(pointer); //freeing memory allocated
	queueCounter[channel-1]--;//decrementing number of elements in queue	
	return data;
}










