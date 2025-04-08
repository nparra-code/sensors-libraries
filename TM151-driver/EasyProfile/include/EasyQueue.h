/**
  * C Style Queue Library
  *
  * @version V1.0 - Nov 10, 2014 - Creation
  * @version ---------- Branch C language ----------
  * @version V1.1 - Sep 02, 2016 - Port to C language
  *
  *
  * @section Example
  * @code
  *     #include "EasyQueue.h"
  *
  *     int main()
  *     {
  *         EasyQueue_Construct();
  *         EasyQueue_Resize(256);
  *         EasyQueue_Push( 123 );
  *         EasyQueue_Push( 456 );
  *         while( ! EasyQueue_Empty() ){
  *             char c = EasyQueue_Front(); // Read first element of the queue
  *             EasyQueue_Pop();            // Delete first element of the queue
  *         }
  *         return 0;
  *     }
  *
  * @endcode
  *
  * @attention
  *          *****        DO NOT CHANGE THIS FILE           *****
  *          ***** Automatically Generated by IMU Assistant *****
  * @attention
  * <h2><center>&copy; COPYRIGHT(c) 2017 SYD Dynamics ApS</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */
#ifndef EASYQUEUE_C_LANGUAGE_H
#define EASYQUEUE_C_LANGUAGE_H


//------------------------------------------------------------------------------
// Configure Parameters
    #define QueueDataType   char      //Define the queue data type
    //#define DEBUG_EASY_QUEUE_
// Configure Parameters
//------------------------------------------------------------------------------


#include <stdlib.h>                   // for calloc(), free()
#ifdef DEBUG_EASY_QUEUE_
#   include <stdio.h>
#endif

void  EasyQueue_Construct(void);

int  EasyQueue_Resize(int bufSize);

int  EasyQueue_Size(void);

void EasyQueue_Destruct(void);

int EasyQueue_Push(QueueDataType data);

int EasyQueue_Empty(void);

QueueDataType EasyQueue_Front(void);

void EasyQueue_Pop(void);

void EasyQueue_Print(void);

#endif // EASYQUEUE_C_LANGUAGE_H
