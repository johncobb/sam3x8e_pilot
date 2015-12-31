/*
 * wan_task.c
 *
 *  Created on: Aug 19, 2015
 *      Author: jcobb
 */

#include <stdint.h>
#include <string.h>


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_usart_serial.h"
#include "sysclk.h"
#include "wan.h"
#include "wan_task.h"

xSemaphoreHandle wan_start_signal = 0;

QueueHandle_t xWanTaskQueue;

static void wan_handler_task(void *pvParameters);
static void wan_queue();

void create_wan_task(uint16_t stack_depth_words, unsigned portBASE_TYPE task_priority)
{

	wan_init();

	vSemaphoreCreateBinary(wan_start_signal);

	xWanTaskQueue = xQueueCreate(10, sizeof(router_msg_t));


//	xTaskQueueRequest = xQueueCreate(10, sizeof(comm_request_t));



	xTaskCreate(	wan_handler_task,			/* The task that implements the command console. */
					(const int8_t *const) "WAN",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					stack_depth_words,					/* The size of the stack allocated to the task. */
					NULL,			/* The parameter is used to pass the already configured USART port into the task. */
					task_priority,						/* The priority allocated to the task. */
					NULL);

}

static uint32_t byte_count = 0;

static void wan_handler_task(void *pvParameters)
{

	while(true) {

		memset(wan_rx_buffer, '\0', WAN_RX_BUFFER_SIZE+1);

		byte_count = wan_handler_async(20);

		if(byte_count > 0) {
			printf("wan bytes_received: %lu\r\n", byte_count);

			printf("wan_rx_buffer: %s\r\n", wan_rx_buffer);

			printf("WAN task...\r\n");
		}
		vTaskDelay(500);

	}
}

static void wan_queue()
{
	static BaseType_t result;
	router_msg_t msg;

	result = xQueueReceive(xWanTaskQueue, &msg, WAN_QUEUE_TICKS);

}
