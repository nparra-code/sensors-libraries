/**
 * Easy Profile (C language interface)
 *
 * @brief  Bridge EasyProfile C++ code to C environment
 *
 * @version 1.1.1 - Sep 01, 2016 - Creation
 *          1.1.2 - Sep 16, 2016 - Add Software Serial Port support for CC254x @ IAR
 *          1.1.3 - Sep 24, 2016 - Add Ep_Calibrated_GyroAcc API
 *          1.1.4 - Aug 24, 2017 - Update according to TM331 FW3.1.8 and 
 *                                 according to TransducerM_Example_CPP_QT_V1-1-9-R.
 *
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
#ifndef EASYPROFILE_C_INTERFACE_H
#define EASYPROFILE_C_INTERFACE_H

#include "EasyObjectDictionary.h"


void EasyProfile_C_Interface_Init(void); 
int  EasyProfile_C_Interface_TX_Request(unsigned int toId, unsigned int cmd, char **txData, int *txSize);
int  EasyProfile_C_Interface_RX(char *rxData, int rxDataSize, Ep_Header *header);

#endif // EASYPROFILE_C_INTERFACE_H

