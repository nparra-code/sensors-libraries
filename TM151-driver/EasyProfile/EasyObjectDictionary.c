/**
 * EasyObjectDictionary.cpp
 * @author COPYRIGHT(c) 2024 SYD Dynamics ApS
 * @see    EasyObjectDictionary.h for more descriptions.
 */
#include "EasyObjectDictionary.h"


//---------------------------------------------------------
// Public Members
Ep_Ack                 ep_Ack;
Ep_Request             ep_Request;
Ep_Status              ep_Status;
Ep_Raw_GyroAccMag      ep_Raw_GyroAccMag;
Ep_Combo               ep_Combo;
Ep_Q_s1_e              ep_Q_s1_e;
Ep_Euler_s1_e          ep_Euler_s1_e;
Ep_RPY                 ep_RPY;
Ep_Gravity             ep_Gravity;
// Public Members
//---------------------------------------------------------



int EasyObjectDictionary_Get_MaxSize(void)
{	
    // Find the maximum size:
    int iDS = 0;
    if( iDS < (int)sizeof(Ep_Ack))                 iDS = sizeof(Ep_Ack);
    if( iDS < (int)sizeof(Ep_Request))             iDS = sizeof(Ep_Request);
    if( iDS < (int)sizeof(Ep_Status))              iDS = sizeof(Ep_Status);
    if( iDS < (int)sizeof(Ep_Raw_GyroAccMag))      iDS = sizeof(Ep_Raw_GyroAccMag);
    if( iDS < (int)sizeof(Ep_Combo))               iDS = sizeof(Ep_Combo);
    if( iDS < (int)sizeof(Ep_Q_s1_e))              iDS = sizeof(Ep_Q_s1_e);
    if( iDS < (int)sizeof(Ep_Euler_s1_e))          iDS = sizeof(Ep_Euler_s1_e);
    if( iDS < (int)sizeof(Ep_RPY))                 iDS = sizeof(Ep_RPY);
    if( iDS < (int)sizeof(Ep_Gravity))             iDS = sizeof(Ep_Gravity);
    return iDS;
}


int EasyObjectDictionary_Write(
    Ep_Header header,
    char     *payloadData, 
    int       payloadSize
){
    int retVal = EP_FAIL_;
    
    switch(header.cmd){
    case EP_CMD_ACK_:{
        if( payloadSize == (signed)(sizeof(Ep_Ack)) ){
            char* dataPtr = (char*)(&ep_Ack);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_STATUS_:{
        if( payloadSize == (signed)(sizeof(Ep_Status)) ){
            char* dataPtr = (char*)(&ep_Status);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_Raw_GYRO_ACC_MAG_:{
        if( payloadSize == (signed)(sizeof(Ep_Raw_GyroAccMag)) ){
            char* dataPtr = (char*)(&ep_Raw_GyroAccMag);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_COMBO_:{
        if( payloadSize == (signed)(sizeof(Ep_Combo)) ){
            char* dataPtr = (char*)(&ep_Combo);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_Q_S1_E_:{
        if( payloadSize == (signed)(sizeof(Ep_Q_s1_e)) ){
            char* dataPtr = (char*)(&ep_Q_s1_e);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_EULER_S1_E_:{
        if( payloadSize == (signed)(sizeof(Ep_Euler_s1_e)) ){
            char* dataPtr = (char*)(&ep_Euler_s1_e);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_RPY_:{
        if( payloadSize == (signed)(sizeof(Ep_RPY)) ){
            char* dataPtr = (char*)(&ep_RPY);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    case EP_CMD_GRAVITY_:{
        if( payloadSize == (signed)(sizeof(Ep_Gravity)) ){
            char* dataPtr = (char*)(&ep_Gravity);
            for(int i=0; i<payloadSize; i++){
              *(dataPtr + i) = *(payloadData + i);
            }
            retVal = EP_SUCC_;
        }
    }break;
    }
    return retVal;
}



