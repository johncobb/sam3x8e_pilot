/*
 * wan.h
 *
 *  Created on: Aug 19, 2015
 *      Author: jcobb
 */

#ifndef WAN_H_
#define WAN_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "freertos_usart_serial.h"

#define WAN_RX_BUFFER_SIZE		(128)

#define WAN_BAUD_RATE           38400


uint8_t wan_init(void);
uint8_t wan_config(void);
void WAN_SEND(uint8_t *cmd);
uint32_t wan_handler_async(uint32_t millis);

typedef struct {
uint8_t messageType;       				// 0x01 for tag
       uint64_t routerMac;              // full MAC or source router
       uint16_t routerShort;            // short id of MAC source router - used for two-way comm
       uint64_t tagMac;                 // full MAC of ble tag
       uint8_t tagConfigSet;            // current configuration set id of tag
       uint8_t tagSerial;               // sequential serial number of tag’s report - used to group pings
       uint16_t tagStatus;              // tag status value (bit mask) within adv data
       uint8_t tagLqi;                  // link quality indicator of tag-to-router signal
       uint8_t tagRssi;                 // signal strength of tag-to-router signal
       uint32_t tagBattery;       		// raw ADC value of battery read - currently not reliable but kept for future uses
       uint32_t tagTemperature;   		// raw ADC value of temp sensor on tag - reserved for future use
} rf_msg_t;


typedef struct {
uint8_t messageType;       // 0x02 for router
       uint64_t routerMac;               // full MAC or source router
       uint16_t routerShort;             // short id of MAC source router - used for two-way comm
       uint8_t routerReset;       // recent reset reason
       uint8_t resetTask;                // task that caused the reset
       uint8_t routerSerial;             // sequential serial number of router msg
       uint8_t routerConfigSet;   // current configuration set id of router
       uint32_t routerMsgCount;   // current tag messages received count since reset
       uint32_t routerUptime;            // current ms since reset
       uint32_t routerBattery;           // raw ADC value of battery read - currently not reliable but kept for future uses
       uint32_t routerTemperature; // raw ADC value of temp sensor on tag - reserved for future use
} router_msg_t;


extern uint8_t wan_rx_buffer[WAN_RX_BUFFER_SIZE+1];

#endif /* WAN_H_ */
