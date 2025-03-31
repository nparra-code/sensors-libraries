/**
 * @file   EasyProtocol.cpp
 * @author COPYRIGHT(c) 2017 SYD Dynamics ApS
 * @see    EasyProtocol.h for more comments
 */
#include "EasyProtocol.h"


//---------------------------------------------------------
// Protected Members
    /**
      * Easy Protocol Core
      */
    static int EasyProtocol_UnWrapInData(char* packageData, 
                                         char**payloadData, int payloadSize);

    /**
     *  Data
     */
    static int     iDS;                   // iDataSize
    static int     oDS;                   // oDataSize
    
    /**
     * Data Frame Wrap & Unwrap Data:
     */
    static char    HEAD_1_;
    static char    HEAD_2_;
    static char    HEAD_LENGTH_;
    static char    HEAD_LENGTH_MSB_AHEAD_;
    static char    SIZE_LENGTH_;
    static int     MAX_PAYLOAD_SIZE_;
    static char    CHECKSUM_LENGTH_;

    /**
      * Receive data auto Round-Up
      */    
    static char    ENABLE_ROUND_UP_;
    static char    ROUND_UP_NUM_;
    static char    roundUpTmp;

    /**
      * CheckSum
      */
    static char    CHECKSUM_OPTION_;
    static uint16  EasyProtocol_Checksum_Generate(char* data, int dataLength);
    static int     EasyProtocol_Checksum_Verify(  char* data, int dataLength, uint16 checksumToBeVerified);
// Protected Members
//---------------------------------------------------------


//---------------------------------------------------------
// Private Members
    /**
      * Statistics
      */
    static int totalMem;
    #ifdef EP_TURN_ON_STATISTICS_
    static unsigned int sRecvByte_Total;  // Total Received Bytes
    static unsigned int sRecvByte_BadHead;
    static unsigned int sRecvByte_GoodHead;
    static unsigned int sRecvPkg_Total;
    static unsigned int sRecvPkg_Omit;
    static unsigned int sRecvPkg_Good;
    static unsigned int sRecvPkg_Bad;
    static unsigned int sSendPkg;
    #endif
    
    /**
      * Easy Protocol Core
      */
    #define EP_PKG_MODIFIER_SIZE_ ( HEAD_LENGTH_ + SIZE_LENGTH_ + CHECKSUM_LENGTH_)  // New in V1.7
    static char* outBuf;
    static char* inBuf;             // Only used in AssembleInData()
    static char* p;                 // Only used in AssembleInData()
    static char omitStream;
    static int  declaredPayloadSize;
// Private Members
//---------------------------------------------------------

    

/**
 * @brief 构造函�?
 */
void EasyProtocol_Construct(void)
{
    totalMem = 0;

    //------------------------------------------------------------------------------
    // Configure Parameters 2/2

        //Default Setting:
#       ifdef SA_PLATFORM_QT5_
        #define EP_QUEUE_SIZE_ (iDS*10)
#       else
        #define EP_QUEUE_SIZE_ (iDS*2)  // Smaller queue size for embedded syst.
#       endif
        HEAD_1_          = (char)0xaa;  totalMem += sizeof(HEAD_1_);
        HEAD_2_          = (char)0x55;  totalMem += sizeof(HEAD_2_);          // This value is not used if HEAD_LENGTH==1
        HEAD_LENGTH_     = 2;     totalMem += sizeof(HEAD_LENGTH_);     // Can be 1 or 2
        SIZE_LENGTH_     = 1;     totalMem += sizeof(SIZE_LENGTH_);     // Can be 1 or 2

        MAX_PAYLOAD_SIZE_= 70;    totalMem += sizeof(MAX_PAYLOAD_SIZE_);

        CHECKSUM_OPTION_ = EP_CHECKSUM_2_BYTES_CRC_; totalMem += sizeof(CHECKSUM_OPTION_);
        /*CHECKSUM_LENGTH_ = 1;*/  totalMem += sizeof(CHECKSUM_LENGTH_);// Can be 1 or 2
        EasyProtocol_SetChecksumOption(CHECKSUM_OPTION_);               //
        HEAD_LENGTH_MSB_AHEAD_= 0;totalMem += sizeof(HEAD_LENGTH_MSB_AHEAD_);
        ENABLE_ROUND_UP_ = 1;     totalMem += sizeof(ENABLE_ROUND_UP_); // Round-up
        ROUND_UP_NUM_    = 4;     totalMem += sizeof(ROUND_UP_NUM_);    // Round-up

        EasyQueue_Construct();    totalMem += (EasyQueue_Size() + 1);   // [v2.2]
    // Configure Parameters 2/2
    //------------------------------------------------------------------------------

    //Default Memory Pointers:
    inBuf = outBuf = 0;
    iDS   = oDS    = 0;

    //Statistics:
#   ifdef EP_TURN_ON_STATISTICS_
    sRecvByte_Total = 0;
    sRecvByte_BadHead = 0;
    sRecvByte_GoodHead = 0;
    sRecvPkg_Total = 0;
    sRecvPkg_Omit = 0;
    sRecvPkg_Good = 0;
    sRecvPkg_Bad = 0;
    sSendPkg = 0;
    totalMem += 8 * sizeof(sRecvByte_Total); // [v2.2] Plus the statistic function occupied memory
#   endif
}


void EasyProtocol_Destruct(void)
{
    if(inBuf)  {free(inBuf);  inBuf=0;}
    if(outBuf) {free(outBuf); outBuf=0;}
    EasyQueue_Destruct();
}


/**
 * @brief   EasyProtocol::Init
 * @return  SUCC_
 *          FAIL_   iDataSize and/or oDataSize too large.
 */
int EasyProtocol_Init(
        int iDataSize,  ///< [INPUT]
        int oDataSize   ///< [INPUT]
){
    //totalMem = 0;                               // [v2.2]
    if((iDS > MAX_PAYLOAD_SIZE_) || (oDS > MAX_PAYLOAD_SIZE_) ) return EP_FAIL_;
    totalMem  -= (EasyQueue_Size() + 1);          // [v2.2]
    if( inBuf )  {free(inBuf);  inBuf = 0;  totalMem -= (iDS+EP_PKG_MODIFIER_SIZE_); }
    if( outBuf ) {free(outBuf); outBuf = 0; totalMem -= (oDS+EP_PKG_MODIFIER_SIZE_); }
    iDS = iDataSize;
    oDS = oDataSize;
    p = inBuf = (char*)calloc((iDS+EP_PKG_MODIFIER_SIZE_), sizeof(char)); totalMem += (iDS+EP_PKG_MODIFIER_SIZE_);
    outBuf    = (char*)calloc((oDS+EP_PKG_MODIFIER_SIZE_), sizeof(char)); totalMem += (oDS+EP_PKG_MODIFIER_SIZE_);
    EasyQueue_Resize(EP_QUEUE_SIZE_);                                     totalMem += (EP_QUEUE_SIZE_ + 1);
    
    if(inBuf && outBuf) return EP_SUCC_;          // [v2.2]
    else                return EP_FAIL_;          // [v2.2]
}



/**
 * @brief EasyProtocol::SetChecksumOption
 * @param option can be one of the following:
 *                 EP_CHECKSUM_1_BYTE_SUM_
 *                 EP_CHECKSUM_2_BYTES_SUM_
 *                 EP_CHECKSUM_2_BYTES_CRC_
 */
void EasyProtocol_SetChecksumOption(char option){
    CHECKSUM_OPTION_ = EP_CHECKSUM_1_BYTE_SUM_;
    CHECKSUM_LENGTH_ = 1;
    if((option == EP_CHECKSUM_2_BYTES_SUM_) ||
            (option == EP_CHECKSUM_2_BYTES_CRC_)){
        CHECKSUM_OPTION_ = option;
        CHECKSUM_LENGTH_ = 2;
    }
}



/**
  * Debug Print
  */
#ifdef EP_TURN_ON_DEBUG_
void  EasyProtocol_DebugPrint(QString title, char* data, int length){
    qDebug()<< " --- " <<title <<" --- length="<<length;
    QDebug* theQDebug = new QDebug(QtDebugMsg);
    for(int i=0; i<length; i++){
        QString t = QString("%1 ").arg((uint8)data[i], 0, 16);
        (*theQDebug) << t;
    }
    delete theQDebug;
    qDebug()<< " --- --- ---";
}
#endif



/**
  * Get Statistics
  */
#ifdef EP_TURN_ON_STATISTICS_
int   EasyProtocol_TotalBytesOfMemoryUsed(){
    return totalMem;
}

int   EasyProtocol_GetStatistic_Recv_Byte_Total(){
    return sRecvByte_Total;
}

float EasyProtocol_GetStatistic_Recv_Byte_BadHeadRate(){
    return ((float)sRecvByte_BadHead / sRecvByte_Total);
}

int   EasyProtocol_GetStatistic_Recv_Pkg_Total(){
    return sRecvPkg_Total;
}

float EasyProtocol_GetStatistic_Recv_Pkg_OmitRate(){
    return  ((float)sRecvPkg_Omit / sRecvPkg_Total);
}

float EasyProtocol_GetStatistic_Recv_Pkg_BadRate(){
    return  ((float)sRecvPkg_Bad / sRecvPkg_Total);
}

float EasyProtocol_GetStatistic_Recv_Pkg_GoodRate(){
    return  ((float)sRecvPkg_Good / sRecvPkg_Total);
}

int   EasyProtocol_GetStatistic_Send_Pkg_Total(){
    return sSendPkg;
}
#endif




int  EasyProtocol_GetInDataMaxSize(void){
    return iDS;
}
int  EasyProtocol_GetOutDataMaxSize(void){
    return oDS;
}


/**
 * @brief EasyProtocol::GetRoundUp
 * @return roundUp parameter
 */
int  EasyProtocol_GetRoundUp(void){
    return roundUpTmp;
}



/**
  * @return  EP_FAIL_    payloadSize too large or easyprotocol setting error.
  *          EP_SUCC_    Creation package successfully.
  */
int EasyProtocol_CreateOutputPackage(
        char*  payloadData, ///< [INPUT]
        int    payloadSize, ///< [INPUT]
        char** packageData, ///< [OUTPUT]
        int*   packageSize  ///< [OUTPUT]
){
    // Check validity:
    if( payloadSize > MAX_PAYLOAD_SIZE_ )               return EP_FAIL_;
    if( (HEAD_LENGTH_!=1) && (HEAD_LENGTH_!=2))         return EP_FAIL_;
    if( (SIZE_LENGTH_!=1) && (SIZE_LENGTH_!=2))         return EP_FAIL_;
    if( (CHECKSUM_LENGTH_!=1) && (CHECKSUM_LENGTH_!=2)) return EP_FAIL_;
    if( 0 == outBuf )                                   return EP_FAIL_;      // [v2.2]

    // Head:
    outBuf[0] = HEAD_1_;
    if(HEAD_LENGTH_==2){
        outBuf[1] = HEAD_2_;
    }

    // Data Length:
    if(SIZE_LENGTH_== 1){
        outBuf[ (int16)HEAD_LENGTH_ ] = (int8)payloadSize;
    }
    else if(SIZE_LENGTH_== 2){
        if(! HEAD_LENGTH_MSB_AHEAD_){
            *((int16*)(outBuf + HEAD_LENGTH_)) = (unsigned int)payloadSize;
        }
        else{
            outBuf[(int16)HEAD_LENGTH_ ]    = (unsigned int)payloadSize >> 8; // MSB
            outBuf[(int16)HEAD_LENGTH_ + 1] = (unsigned int)payloadSize;      // LSB
        }
    }

    // Payload:
    char *outBufTmp = outBuf + HEAD_LENGTH_ + SIZE_LENGTH_;
    for(int t=0; t<payloadSize; t++){
        outBufTmp[t] = payloadData[t];
    }

    // Results:
    *packageData = outBuf;
    *packageSize = payloadSize + EP_PKG_MODIFIER_SIZE_;

    // CheckSum:
    uint16 checkSum = EasyProtocol_Checksum_Generate(outBuf + HEAD_LENGTH_, payloadSize + SIZE_LENGTH_);
    if(CHECKSUM_LENGTH_ == 1) {
        outBuf[*packageSize - CHECKSUM_LENGTH_] = checkSum;
    }
    else if(CHECKSUM_LENGTH_ == 2){
        for(int i=0; i<2; i++){
            *(((uint8_t*)(outBuf + *packageSize - CHECKSUM_LENGTH_)) + i) = *(((uint8_t*)(&checkSum))+i);
        }
    }

#   ifdef EP_TURN_ON_DEBUG_
    EasyProtocol_DebugPrint("CreateOutputPackage()", *packageData, *packageSize);
#   endif

    // Statistics:
#   ifdef EP_TURN_ON_STATISTICS_
    sSendPkg++;
#   endif
    return EP_SUCC_;
}


/**
 * @brief EasyProtocol::Checksum_Generate
 */
uint16 EasyProtocol_Checksum_Generate(
        char* data,       ///< [INPUT]
        int   dataLength  ///< [INPUT]
){
    uint16 checkSum = 0;

    if((CHECKSUM_OPTION_ == EP_CHECKSUM_1_BYTE_SUM_) ||
            (CHECKSUM_OPTION_ == EP_CHECKSUM_2_BYTES_SUM_) ){
        int i;
        for( i=0; i<dataLength; i++){
            checkSum += (unsigned)( * ((uint8*)(data + i)) );
        }
    }
    else{ //EP_CHECKSUM_2_BYTES_CRC_   CRC-16 (Modbus)
        unsigned char*d = (unsigned char*)data;
        unsigned char c;
        checkSum = 0xffff;
        while(dataLength--){
            unsigned int i = 8;
            checkSum ^= (unsigned int)(*(d++));
            while(i--){
                c = checkSum & 0x0001;
                checkSum >>= 1;
                if(c) checkSum ^= 0xa001;
            }
        }
    }

    return checkSum;
}


/**
 * @brief    EasyProtocol::Checksum_Verify
 * @return   EP_SUCC_    check passed
 *           EP_FAIL_    check failed
 */
int EasyProtocol_Checksum_Verify(
        char*  data,                  ///< [INPUT]
        int    dataLength,            ///< [INPUT]
        uint16 checksumToBeVerified   ///< [INPUT]
){
    uint16 checkSum = EasyProtocol_Checksum_Generate(data, dataLength);

    if(CHECKSUM_LENGTH_ == 1){
        checkSum = (uint8)checkSum;
    }
#   ifdef EP_TURN_ON_DEBUG_
    qDebug()<<"EasyProtocol: calcChecksum" << checkSum;
#   endif
    if(checksumToBeVerified == checkSum) return EP_SUCC_;
    else return EP_FAIL_;
}


/**
 * @brief  EasyProtocol::UnWrapInData
 * @return EP_FAIL_
 *         EP_SUCC_
 */
int EasyProtocol_UnWrapInData(
    char* packageData,  ///< [INPUT]
    char**payloadData,  ///< [OUTPUT]
    int   payloadSize   ///< [INPUT]
){
#   ifdef EP_TURN_ON_DEBUG_
    EasyProtocol_DebugPrint("UnWrapInData() ", packageData + HEAD_LENGTH_ + SIZE_LENGTH_, payloadSize);
#   endif

    // CheckSum Verify:
    uint16 recvChecksum = 0;

    if(CHECKSUM_LENGTH_ == 1) {
        recvChecksum = packageData[payloadSize + EP_PKG_MODIFIER_SIZE_- CHECKSUM_LENGTH_];
    }
    else if(CHECKSUM_LENGTH_ == 2){
        for(int i=0; i<2; i++){
            *(((uint8_t*)(&recvChecksum))+i) =  *(((uint8_t*)(packageData + payloadSize + EP_PKG_MODIFIER_SIZE_ - CHECKSUM_LENGTH_)) + i);
        }
    }

#   ifdef EP_TURN_ON_DEBUG_
    qDebug()<<"EasyProtocol: recvChecksum" << recvChecksum;
#   endif

    //if( ! Checksum_Verify(packageData, payloadSize + EP_PKG_MARKUP_SIZE_ - CHECKSUM_LENGTH_ - roundUp, recvChecksum) ){
    if( EP_SUCC_ != EasyProtocol_Checksum_Verify(packageData+HEAD_LENGTH_, payloadSize + SIZE_LENGTH_, recvChecksum) ){
#       ifdef EP_TURN_ON_STATISTICS_
        sRecvPkg_Bad++;
#       endif
        *payloadData = 0;
        return EP_FAIL_;
    }
    else {
        ////Verify Passed, Save data:
        //for(int t=0; t<iDS; t++){
        //    iData[t+roundUp] = *(packageData+HEAD_LENGTH_+SIZE_LENGTH_+t);
        // }
        *payloadData = packageData + HEAD_LENGTH_ + SIZE_LENGTH_;
#       ifdef EP_TURN_ON_STATISTICS_
        sRecvPkg_Good++;
#       endif
        return EP_SUCC_;
    }
}


/**
 * EasyProtocol_AssembleInputPackage
 * @return EP_NORMAL_EXIT_
 *         EP_FAIL_
 *         EP_SUCC_
 *         EP_QUEUE_EMPTY_
 */
int EasyProtocol_AssembleInputPackage(
    char  *rawData,      ///< [INPUT]
    int    rawDataLenth, ///< [INPUT]
    char **payloadData,  ///< [OUTPUT]
    int   *payloadSize   ///< [OUTPUT]
){
    char newDataReceived = 0;

    // Check validity:
    if( (HEAD_LENGTH_!=1) && (HEAD_LENGTH_!=2))         return EP_FAIL_;
    if( (SIZE_LENGTH_!=1) && (SIZE_LENGTH_!=2))         return EP_FAIL_;
    if( (CHECKSUM_LENGTH_!=1) && (CHECKSUM_LENGTH_!=2)) return EP_FAIL_;
    if( 0 == inBuf )                                    return EP_FAIL_;      // [v2.2]
    
#   ifdef EP_TURN_ON_STATISTICS_
    sRecvByte_Total += rawDataLenth;
#   endif

    for(int i=0; i<rawDataLenth; i++){
        EasyQueue_Push( *(rawData+i) );
    }

    while( ! EasyQueue_Empty() ){
        char c = EasyQueue_Front();
        EasyQueue_Pop();

        //Wait for valid Head:
        if( p == inBuf ){
            if( c == (char)HEAD_1_ ){
                *p++ = c;
                omitStream = 'N';
#               ifdef EP_TURN_ON_STATISTICS_
                sRecvByte_GoodHead++;
#               endif
            }
#           ifdef EP_TURN_ON_STATISTICS_
            else{
                sRecvByte_BadHead++;
            }
#           endif
        }
        else if( (HEAD_LENGTH_ == 2) && (p == (inBuf+1)) ){
            if(c == (char)HEAD_2_)
                *p++ = c;
            else
                p=inBuf;
        }
        else if((p >= (inBuf + HEAD_LENGTH_)) &&
                (p <  (inBuf + HEAD_LENGTH_ + SIZE_LENGTH_))){
            *p++ = c;

            if( p >= (inBuf + HEAD_LENGTH_ + SIZE_LENGTH_)){
                if(SIZE_LENGTH_ == 1){
                    declaredPayloadSize = *((uint8 *)(p-SIZE_LENGTH_));
                }
                else if(SIZE_LENGTH_ == 2){
                    if(! HEAD_LENGTH_MSB_AHEAD_){
                        declaredPayloadSize = *((uint16 *)(p-SIZE_LENGTH_));
                    }
                    else{
                        declaredPayloadSize =
                                ((uint16)(*((uint8 *)(p-SIZE_LENGTH_)))<<8) +
                                (uint16)(*((uint8 *)(p-SIZE_LENGTH_+1)));
                    }
                }

                if((declaredPayloadSize > MAX_PAYLOAD_SIZE_) || (declaredPayloadSize<=0)){
                    p = inBuf;
                    continue;
                }

#               ifdef EP_TURN_ON_DEBUG_
                qDebug()<<"declaredPayloadSize="<<declaredPayloadSize;
                qDebug()<<"iDS="<<iDS;
                qDebug()<<"MAX_PAYLOAD_SIZE_="<<MAX_PAYLOAD_SIZE_;
#               endif

                roundUpTmp = 0;
                if(ENABLE_ROUND_UP_){
                    if( (int32)declaredPayloadSize % ((int32)ROUND_UP_NUM_)){
                        roundUpTmp = ((int32)ROUND_UP_NUM_) - ((int32)declaredPayloadSize % ((int32)ROUND_UP_NUM_));
                    }
                }
#               ifdef EP_TURN_ON_DEBUG_
                qDebug()<<"roundUpTmp="<<(int16)roundUpTmp;
#               endif

                if( (declaredPayloadSize /*+ roundUpTmp*/) <= iDS){
                    omitStream = 'N';
#                   ifdef EP_TURN_ON_DEBUG_
                    qDebug()<<"omitStream = 'N'";
#                   endif
                }
                else{
                    omitStream = 'Y';
#                   ifdef EP_TURN_ON_DEBUG_
                    qDebug()<<"omitStream = 'Y'";
#                   endif
                }
            }
        }
        else {
            if(omitStream == 'N'){
                *p++ = c;  // Read Useful Data
                if( p >= (inBuf + declaredPayloadSize +  EP_PKG_MODIFIER_SIZE_) ) {
#                   ifdef EP_TURN_ON_STATISTICS_
                    sRecvPkg_Total++;
#                   endif
                    p = inBuf;
                    newDataReceived = 1;
                    break;
                }
            }
            else{
                p++; // Omit unrelevant data
                if( p >= (inBuf + declaredPayloadSize + EP_PKG_MODIFIER_SIZE_) ){
#                   ifdef EP_TURN_ON_STATISTICS_
                    sRecvPkg_Total++;
                    sRecvPkg_Omit++;
#                   endif
                    p = inBuf;
                    break;
                }
            }
        }
    }

    if( newDataReceived ){
        if( EP_SUCC_ == EasyProtocol_UnWrapInData(inBuf, payloadData, declaredPayloadSize) ){
            *payloadSize = declaredPayloadSize;
            return EP_SUCC_;
        }
        return EP_FAIL_;
    }
    if(EasyQueue_Empty()){
        *payloadData = 0;
        *payloadSize = 0;
        return EP_QUEUE_EMPTY_;
    }
    else{
        *payloadData = 0;
        *payloadSize = 0;
        return EP_NORMAL_EXIT_;
    }
}

