/**
 * Easy Object Dictionary
 *
 * @brief   Defines and Handles all data structure typies for communicaiton over e.g. EasyProtocol
 *
 * @version 1.1.8     - Jul 17, 2016
 *          1.1.8 (R) - Jul 27, 2016 - Release Version.
 *          ------------ Branch C Language ------------
 *          1.2.1     - Sep 02, 2016 - Port to C Language.
 *          1.2.2     - Sep 24, 2016 - Add Ep_Calibrated_GyroAcc.
 *          1.2.3     - Dec 11, 2016 - Add uuid into Ep_Calibrated_GyroAcc.
 *          1.2.4 (R) - Aug 24, 2017 - Update according to TM331 FW3.1.8 and 
 *                                     according to TransducerM_Example_CPP_QT_V1-1-9-R.
 *          1.2.5 (R) - Mar 19, 2024 - Ep_Ack and Ep_Request add reserved field.
 *                                     Add EP_CMD_COMBO_. Update Ep_Status definition.
 *
 *
 * @attention
 *          *****        DO NOT CHANGE THIS FILE           *****
 *
 *
 * @attention
 * <h2><center>&copy; COPYRIGHT(c) 2024 SYD Dynamics ApS</center></h2>
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
#ifndef EASYOBJECTDICTIONARY_H
#define EASYOBJECTDICTIONARY_H

#include "BasicTypes.h"

//--------------------------------------------------------------
// Header
#define EP_CMD_TYPE_          uint8
#define EP_CMD_BITS_            (7)
#define EP_CMD_MASK_  (0x0000007fu)
#define EP_QOS_TYPE_          uint8
#define EP_QOS_BITS_            (3)
#define EP_QOS_MASK_  (0x00000007u)
#define EP_ID_TYPE_          uint16
#define EP_ID_BITS_            (11)
#define EP_ID_MASK_   (0x000007ffu)

typedef struct{
    uint32      cmd  : EP_CMD_BITS_;      // Command Identifier (e.g. EP_CMD_RPY_ is one of the possible Command Identifier).
    uint32      qos  : EP_QOS_BITS_;      // Quality-of-Service of the source device (only available on certain Models).
    uint32    fromId : EP_ID_BITS_;       // The Device Short-ID indicating the source of the data being delivered.
    uint32      toId : EP_ID_BITS_;       // The Device Short-ID indicating the destination of the data being delivered.
} Ep_Header;
// Header
//--------------------------------------------------------------

    
//--------------------------------------------------------------
// Common ID Definition
#define EP_ID_BROADCAST_          ((EP_ID_TYPE_) 0x0000)
#define EP_ID_UNSPECIFIED_        ((EP_ID_TYPE_) 0x0001)
#define EP_ID_HOST_               ((EP_ID_TYPE_) 0x0002)  // HOST refers to PCs or similar
                                                          //   e.g. the Host is the PC running ImuAssistant
// Common ID Definition
//--------------------------------------------------------------


#define global_SysShortId         (EP_ID_HOST_)           // Set the current device to HOST
#define global_SysQoS             (0)                     // 0 means QoS not available.


//--------------------------------------------------------------
// Command Identifier Definition
#define EP_CMD_REQUEST_           ((EP_CMD_TYPE_)12)
#define EP_CMD_ACK_               ((EP_CMD_TYPE_)13)

#define EP_CMD_STATUS_            ((EP_CMD_TYPE_)22)

#define EP_CMD_Q_S1_S_            ((EP_CMD_TYPE_)31)
#define EP_CMD_Q_S1_E_            ((EP_CMD_TYPE_)32)
#define EP_CMD_EULER_S1_S_        ((EP_CMD_TYPE_)33)
#define EP_CMD_EULER_S1_E_        ((EP_CMD_TYPE_)34)
#define EP_CMD_RPY_               ((EP_CMD_TYPE_)35)
#define EP_CMD_GRAVITY_           ((EP_CMD_TYPE_)36)

#define EP_CMD_Raw_GYRO_ACC_MAG_  ((EP_CMD_TYPE_)41)
#define EP_CMD_COMBO_             ((EP_CMD_TYPE_)43)
// Command Identifier Definition
//--------------------------------------------------------------


typedef struct{
    Ep_Header         header;
    EP_CMD_TYPE_      cmdAck;      // The command it acknowledges to
    uint8        reserved[3];
} Ep_Ack;

typedef struct{
    Ep_Header         header;
    EP_CMD_TYPE_  cmdRequest;      // The command requested
    uint8        reserved[3];
} Ep_Request;

typedef union{
    uint16 all_Bits;
    struct{
        uint16 qos  : 3;  // Quality of Service.
                          // 0: Service unavailable due to boot or in sleep mode.
                          // 1: Service unavailable due to System Fault.
                          // 2: Limited Service -- Some functions are not available / Very limited measurement accuracy
                          //                    (e.g. After Dynamic Boot)
                          // 3: Basic Service   -- All functions available and providing basic performance.
                          //                    (e.g. After Static Boot)
                          // 4: Fine Service    -- All function available and providing fine performance.
                          //                    (e.g. DynamicGyroCalib success 30 seconds after boot)
                          // 5: Very Good Service -- All function available and providing very good performance.
                          //                    (e.g. Two DynamicGyroCalib success after 5 minutes)
                          // 6: Reserved.
                          // 7: Use extended QoS definition.
        uint16      :13;  // Unused Bits
    }bits;
} Ep_Status_SysState;

typedef struct {
    Ep_Header            header;
    uint32            timeStamp;   // Timestamp              (Unit: uS)
    float32         temperature;   // Sensor temperature     (Unit: Celcius)
    uint16           updateRate;   // Internal sampling rate (Unit: Hz)
    Ep_Status_SysState sysState;   // System State
} Ep_Status;

typedef struct{
    Ep_Header      header;
    uint32      timeStamp;         // Timestamp when the data is sampled (Unit: uS)
    float32       gyro[3];         // Gyro raw data         (Unit: rad/s)
    float32        acc[3];         // Accel raw data        (Unit: g)
    float32        mag[3];         // Magnetometer raw data (Unit: one earth magnetic field)
} Ep_Raw_GyroAccMag;

typedef struct{
    Ep_Header            header;
    uint32            timeStamp;   // Timestamp when the data is sampled (Unit: uS)
    Ep_Status_SysState sysState;   // It includes Quality of Service (Refer to the above Ep_Status_SysState definition)
    int16          roll;           // Unit: 0.01 degree
    int16          pitch;          // Unit: 0.01 degree
    uint16         yaw;            // Unit: 0.01 degree
    int32          q1;             // Quaternion in (w,x,y,z)format. Must convert unit before use: q1*(1e-7f)
    int32          q2;             // Quaternion in (w,x,y,z)format. Must convert unit before use: q2*(1e-7f)
    int32          q3;             // Quaternion in (w,x,y,z)format. Must convert unit before use: q3*(1e-7f)
    int32          q4;             // Quaternion in (w,x,y,z)format. Must convert unit before use: q4*(1e-7f)
    int32          wx;             // Gyroscope. Unit: 0.00001 rad/s 
    int32          wy;             // Gyroscope. Unit: 0.00001 rad/s 
    int32          wz;             // Gyroscope. Unit: 0.00001 rad/s 
    int32          ax;             // Accelerometer. Unit: 0.00001 g   ( 1g = 9.794m/(s^2) )
    int32          ay;             // Accelerometer. Unit: 0.00001 g   ( 1g = 9.794m/(s^2) )
    int32          az;             // Accelerometer. Unit: 0.00001 g   ( 1g = 9.794m/(s^2) )
    int16          mx;             // Must convert unit before use: mx*(1e-3f)
    int16          my;             // Must convert unit before use: my*(1e-3f)
    int16          mz;             // Must convert unit before use: mz*(1e-3f)
    int8           temperature;    // Unit: Celcius
    uint8          updateRate;     // Unit: 10 Hz
    uint16         reserved1;      
    uint16         simpleChecksum; // Sum of the Ep_Combo data structure byte per byte except simpleChecksum itself. (see main_example.c for how to use it)
} Ep_Combo;

typedef struct{
    Ep_Header      header;
    uint32      timeStamp;         // Timestamp when the Quaternion is calculated (Unit: uS)
    float32          q[4];         // Quaternion representing the rotation from the current sensor frame to the Earth frame.
} Ep_Q_s1_e;

typedef struct{
    Ep_Header      header;
    uint32      timeStamp;         // Timestamp when the Eular angles are calculated (Unit: uS)
    float32           psi;         // Eular angle (psi)   from the current sensor frame to the Earth frame (Unit: degree)
    float32         theta;         // Eular angle (theta) from the current sensor frame to the Earth frame (Unit: degree)
    float32           phi;         // Eular angle (phi)   from the current sensor frame to the Earth frame (Unit: degree)
} Ep_Euler_s1_e;

typedef struct{
    Ep_Header      header;
    uint32      timeStamp;          // Timestamp when the Roll Pitch Yaw angles are calculated (Unit: uS)
    float32          roll;          // Roll                (Unit: degree)
    float32         pitch;          // Pitch               (Unit: degree)
    float32           yaw;          // Yaw (i.e. Heading)  (Unit: degree)
} Ep_RPY;

typedef struct{
    Ep_Header      header;
    uint32      timeStamp;          // Timestamp when the Gravity Vector is calculated (Unit: uS)
    float32          g[3];          // (g[0], g[1], g[2]) represents the vector of Earth Gravity in the sensor frame (Unit: g)
} Ep_Gravity;


// Maximum Size of Object Data:
int EasyObjectDictionary_Get_MaxSize(void);
int EasyObjectDictionary_Write(Ep_Header header, char *payloadData, int payloadSize);

extern Ep_Ack                 ep_Ack;
extern Ep_Request             ep_Request;
extern Ep_Status              ep_Status;
extern Ep_Raw_GyroAccMag      ep_Raw_GyroAccMag;
extern Ep_Combo               ep_Combo;
extern Ep_Q_s1_e              ep_Q_s1_e;
extern Ep_Euler_s1_e          ep_Euler_s1_e;
extern Ep_RPY                 ep_RPY;
extern Ep_Gravity             ep_Gravity;

#endif // EASYOBJECTDICTIONARY_H
