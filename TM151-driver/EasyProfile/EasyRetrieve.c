#include "EasyRetrieve.h"

void SerialPort_DataGet(uart_t* myUART){

	char* rxData[10]; 
	int   rxSize=10;
    
    uart_read(myUART, (uint8_t*)rxData, (size_t)rxSize, 1000); // Read the data from UART

	Ep_Header header; // Then let the EasyProfile do the rest such as data assembling and checksum verification.
    if( EP_SUCC_ == EasyProfile_C_Interface_RX((char*)rxData, (int)rxSize, &header)){

        ESP_LOGI("EasyProfile", "RX header CMD: %d", header.cmd);
        
        switch (header.cmd) { // The program will only reach this line if and only if a correct and complete package has received.
        case EP_CMD_ACK_:{

        }break;
        case EP_CMD_STATUS_:{

        }break;
        case EP_CMD_COMBO_:{
            // Accelerometer:
            float ax = (ep_Combo.ax)*(1e-5f);                     // Unit: 1g, 1g = 9.794m/(s^2)
            float ay = (ep_Combo.ay)*(1e-5f);
            float az = (ep_Combo.az)*(1e-5f);
            // Gyroscope:
            float wx = (ep_Combo.wx)*(1e-5f);                     // Unit: rad/s
            float wy = (ep_Combo.wy)*(1e-5f);
            float wz = (ep_Combo.wz)*(1e-5f);
            // Magnetometer:
            float mx = (ep_Combo.mx)*(1e-3f);                     // Unit: one earth magnetic field
            float my = (ep_Combo.my)*(1e-3f);                     // vector (mx, my, mz) is used as direction reference of the local magnetic field.
            float mz = (ep_Combo.mz)*(1e-3f);                     // The norm(mx, my, mz) may not be accurate.
            // Quaternion in (w,x,y,z) format
            float q1 = (ep_Combo.q1)*(1e-7f);
            float q2 = (ep_Combo.q2)*(1e-7f);
            float q3 = (ep_Combo.q3)*(1e-7f);
            float q4 = (ep_Combo.q4)*(1e-7f);
            // RPY:
            float roll  = (ep_Combo.roll)*(1e-2f);                // Unit: degree
            float pitch = (ep_Combo.pitch)*(1e-2f);               // Unit: degree
            float yaw   = (ep_Combo.yaw)*(1e-2f);                 // Unit: degree

            printf("RawAcc %f %f %f \n", ax, ay, az);
            printf("RawGyro%f %f %f \n", wx, wy, wz);
            printf("RawMag %f %f %f \n", mx, my, mz);
            printf("Q %f %f %f %f\n", q1, q2, q3, q4);
            printf("RPY %f %f %f\n", roll, pitch, yaw);

        }break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:{              // Here we demonstrate a few examples on how to use the received data
            // Raw Data received
            unsigned int timeStamp = ep_Raw_GyroAccMag.timeStamp;
            float ax = ep_Raw_GyroAccMag.acc[0];     // Note 1: ep_Raw_GyroAccMag is defined in the EasyProfile library as a global variable
            float ay = ep_Raw_GyroAccMag.acc[1];     // Note 2: for the units and meaning of each value, refer to EasyObjectDictionary.h
            float az = ep_Raw_GyroAccMag.acc[2];
            float wx = ep_Raw_GyroAccMag.gyro[0];
            float wy = ep_Raw_GyroAccMag.gyro[1];
            float wz = ep_Raw_GyroAccMag.gyro[2];
            float mx = ep_Raw_GyroAccMag.mag[0];
            float my = ep_Raw_GyroAccMag.mag[1];
            float mz = ep_Raw_GyroAccMag.mag[2];
            // use raw data here...
            // ...
            printf("RawAcc %f %f %f \n", ax, ay, az);
            printf("RawGyro%f %f %f \n", wx, wy, wz);
            printf("RawMag %f %f %f \n", mx, my, mz);
        }break;
        case EP_CMD_Q_S1_E_:{
            // Quanternion received
            unsigned int timeStamp = ep_Q_s1_e.timeStamp;
            float q0 = ep_Q_s1_e.q[0];              // Note 1, ep_Q_s1_e is defined in the EasyProfile library as a global variable
            float q1 = ep_Q_s1_e.q[1];              // Note 2, for the units and meaning of each value, refer to EasyObjectDictionary.h
            float q2 = ep_Q_s1_e.q[2];
            float q3 = ep_Q_s1_e.q[3];
            // used q1 q2 q3 q4 here...
            // ...
            printf("Q %f %f %f %f\n", q0, q1, q2, q3);
        }break;
        case EP_CMD_EULER_S1_E_:{

        }break;
        case EP_CMD_RPY_:{
            // Roll Pitch Yaw data received
            unsigned int timeStamp = ep_RPY.timeStamp;
            float roll  = ep_RPY.roll;        // Note 1, ep_RPY is defined in the EasyProfile library as a global variable
            float pitch = ep_RPY.pitch;       // Note 2, for the units and meaning of each value, refer to EasyObjectDictionary.h
            float yaw   = ep_RPY.yaw;
            // Use roll, pitch, yaw data here...
            // ...
            printf("RPY %f %f %f\n", roll, pitch, yaw);
        }break;
        case EP_CMD_GRAVITY_:{

        }break;
        }
    }
}