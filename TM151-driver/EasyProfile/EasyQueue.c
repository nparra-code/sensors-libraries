/**
 * @file   EasyQueue.c
 * @author COPYRIGHT(c) 2017 SYD Dynamics ApS
 * @see    EasyQueue.h for more comments
 */
#include "EasyQueue.h"

//---------------------------------------------------------
// Private Members
static QueueDataType* queueData;
static int queueSize;
static int queueStart, queueEnd;
// Private Members
//---------------------------------------------------------



void EasyQueue_Construct(void)
{
    queueSize = 16+1;
    queueStart= 0;
    queueEnd  = 0;
    queueData = (QueueDataType*)calloc(queueSize, sizeof(QueueDataType) );
}


int EasyQueue_Resize(int bufSize)
{
    queueSize = bufSize+1;
    queueStart= 0;
    queueEnd  = 0;
    queueData = (QueueDataType*)realloc(queueData, queueSize * sizeof(QueueDataType));
    if(queueData) return 1;
    else return 0;
}


int  EasyQueue_Size(void){
    return (queueSize-1);
}


void EasyQueue_Destruct(void){
    free(queueData);
}


int EasyQueue_Push(QueueDataType data){
    int full = 0;

    queueData[queueEnd++] = data;

    if(queueEnd >= queueSize) queueEnd = 0;
    if(queueEnd == queueStart){
        full = 1;
        if((++queueStart) >= queueSize) queueStart = 0;
    }
    return full;
}


int EasyQueue_Empty(void){
    if(queueEnd == queueStart) return 1;
    else return 0;
}


QueueDataType EasyQueue_Front(void){
    return queueData[queueStart];
}


void EasyQueue_Pop(void){
    if(EasyQueue_Empty()) return;
    if((++queueStart) >= queueSize) queueStart = 0;
}


void EasyQueue_Print(void){
#   ifdef DEBUG_EASY_QUEUE_
    int i;
    printf("\nQueue: ");

    for(i=0; i<queueSize; i++){
        if(i==queueStart)
            printf(" [");
        printf(" %d ", (int)(queueData[i]));
        if(i==queueEnd)
            printf("] ");
    }

    if( Queue_Empty() ){
        printf(" Empty ");
    }
    printf("\n ");
#   endif
}

