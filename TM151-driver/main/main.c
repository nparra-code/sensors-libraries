// ---------------------------------- INSTRUCTIONS ----------------------------------------
// To use the EasyProfile library to communicate with TransdcuerM,
// you need to copy the entire EasyProfile folder into your target system project folder,
// And then follow the steps below.
// Please note that files within the EasyProfile folder normally should not be modified, 
// and all you need to do is to follow the steps described within this file. 
//------------------------------------------------------------------------------------------
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include "EasyObjectDictionary.h"
#include "EasyProfile.h"

#include "platform_esp32s3.h" // Include the platform-specific header file for ESP32-S3
#include "EasyRetrieve.h"
#include <portmacro.h>

#define UART_TX 17 // GPIO pin for UART TX
#define UART_RX 18 // GPIO pin for UART RX

uart_t myUART;

void app_main(void)
{

    if (uart_init_with_defaults(&myUART, 115200, 1024, UART_TX, UART_RX) == 0) {
        printf("UART initialized successfully\n");
    } else {
        printf("UART initialization failed\n");
    }


    // Step 2: Initialization:
	EasyProfile_C_Interface_Init();

	while(1){
		
		// Step 3 and Step 4 are optional, only if you want to use the request-response communication pattern
		// Step 3: Request Roll Pitch Yaw Data from TransdcuerM 
	    uint16 toId = EP_ID_BROADCAST_;
		char*  txData;
		int    txSize;

        if(EP_SUCC_ == EasyProfile_C_Interface_TX_Request(toId, EP_CMD_RPY_, &txData, &txSize)){
            // You can request a different data type by changing the EP_CMD_RPY_ to some other value defined in EasyObjectDictionary.h
            uart_write(&myUART, (uint8_t*)txData, (size_t)txSize); // write the data to UART
    
        }
		
		SerialPort_DataGet(&myUART); // Call the function to handle incoming data

		vTaskDelay(200/portTICK_PERIOD_MS);  // Please avoid sending requests too often.
        
	}
}
