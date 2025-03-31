/**
 * EasyProfile_C_Interface.cpp
 * @author COPYRIGHT(c) 2017 SYD Dynamics ApS
 * @see    EasyProfile.h for more descriptions.
 */
#include "EasyProfile.h"
#include "EasyProtocol.h"


void EasyProfile_C_Interface_Init(void){
    int maxSize = EasyObjectDictionary_Get_MaxSize();
    EasyProtocol_Construct();
    EasyProtocol_Init(maxSize, maxSize);
}


//----------------------------------------------------------------------------------------
// Serial Send Data      
/**
 * Requesting data from the Sensor Module.
 */
int  EasyProfile_C_Interface_TX_Request(
    unsigned int toId, ///< [INPUT] Specify which device should respond to the request
                       ///          The toId number is defined in ImuAssistant as 'ShortId'
                       ///          You can also choose to broadcast the request without
                       ///          specifying the target device ID by using the Macro 'EP_ID_BROADCAST_'
    unsigned int cmd,  ///< [INPUT] Specify data type to be requested.
                       ///          Examples are: EP_CMD_RPY_
                       ///                        EP_CMD_Q_S1_S_
                       ///                        EP_CMD_Raw_GYRO_ACC_MAG_
    char**    txData,  ///< [OUTPUT] The pointer to the generated package buffer which should be sent out.
    int*      txSize   ///< [OUTPUT] The size of the generated package buffer.
){
    ep_Request.header.cmd    = EP_CMD_REQUEST_;
    ep_Request.header.qos    = 0;
    ep_Request.header.fromId = (EP_ID_TYPE_)global_SysShortId;
    ep_Request.header.toId   = (EP_ID_TYPE_)toId;
    ep_Request.cmdRequest    = (EP_ID_TYPE_)cmd;
    
    
    if(EP_SUCC_ == EasyProtocol_CreateOutputPackage( 
                        (char*)(&ep_Request), sizeof(Ep_Request),
                        txData, txSize)
    ){
        return EP_SUCC_;
    }
    return EP_FAIL_;
}
// Serial Send Data
//----------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------
// Serial Data Receive
/**
 * Call this function when new serial data arrives
 * @return EP_SUCC_:           New complete package received.
 *         Other return value: No new complete package received.
 */
int EasyProfile_C_Interface_RX(
    char *rxData,        ///< [INPUT]  Pointer to received raw data
    int   rxDataSize,    ///< [INPUT]  Received raw data size
    Ep_Header *header    ///< [OUTPUT] The received data package Header
){
    if((rxDataSize <= 0) || (rxData == 0) || (header == 0)) return EP_FAIL_;
    
    char * payloadData;
    int    payloadSize;
    int retVal = EasyProtocol_AssembleInputPackage(
                      rxData, rxDataSize,
                      &payloadData, &payloadSize);
    
    if((retVal == EP_SUCC_) && payloadData){
        // New data received...
        
        // Read Header:
        if(payloadSize < (signed)(sizeof(Ep_Header))) return EP_FAIL_;
        for(int i=0; i < ((signed)sizeof(Ep_Header)); i++){
            *(((char*)(header))+i) = payloadData[i];
        }
        
        // Save data:
        return EasyObjectDictionary_Write(*header, payloadData, payloadSize);
    }
    
    return retVal;
}
// Serial Data Receive
//----------------------------------------------------------------------------------------


