/*
 * app_task.c
 *
 *  Created on: Jul 22, 2015
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
#include "tcpip.h"
#include "app_task.h"
#include "imu.h"
#include "MadgwickAHRS.h"

xSemaphoreHandle app_start_signal = 0;

static volatile bool start_task = false;
static void app_handler_task(void *pvParameters);


void create_app_task(uint16_t stack_depth_words, unsigned portBASE_TYPE task_priority)
{
	vSemaphoreCreateBinary(app_start_signal);

	xTaskCreate(	app_handler_task,			/* The task that implements the command console. */
					(const int8_t *const) "APP",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					stack_depth_words,					/* The size of the stack allocated to the task. */
					NULL,			/* The parameter is used to pass the already configured USART port into the task. */
					task_priority,						/* The priority allocated to the task. */
					NULL);

}

void app_start(void)
{

	printf("app_start\r\n");
	if(xSemaphoreTake(app_start_signal, portMAX_DELAY)) {
		start_task = true;
	}
}


int16_t axis_rotation[3];
int16_t axis_acceleration[3];
int16_t axis_mag[3];

static void app_handler_task(void *pvParameters)
{

	imu_init();

	printf("imu test: %d\r\n", imu_test_connection() == 1);
	vTaskDelay(10);

	printf("imu running...\r\n");
	while(true) {


//		imu_get_rotation(&axis_rotation[0], &axis_rotation[1], &axis_rotation[2]);
//		printf("axis_rotation[x,y,z]: %d %d %d\r\n", axis_rotation[0], axis_rotation[1], axis_rotation[2]);

//		imu_get_acceleration(&axis_acceleration[0], &axis_acceleration[1], &axis_acceleration[2]);
//		printf("axis_acceleration[x,y,z]: %d %d %d\r\n", axis_acceleration[0], axis_acceleration[1], axis_acceleration[2]);

//		imu_get_mag(&axis_mag[0], &axis_mag[1], &axis_mag[2]);
//		printf("m:xyz %d %d %d\r\n", axis_mag[0], axis_mag[1], axis_mag[2]);

		int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
//		imu_getmotion6(&ax, &ay, &az, &gx, &gy, &gz);
//		printf("a:xyz %d %d %d g:xyz %d %d %d\r\n", ax, ay, az, gx, gy, gz);



		imu_getmotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//		printf("a:xyz %d %d %d g:xyz %d %d %d m:xyz %d %d %d\r\n", ax, ay, az, gx, gy, gz, mx, my, mz);

		madgwick_ahrs_update(gx, gy, gz, ax, ay, az, mx, my, mz);

		printf("q0: %f q1: %f q2: %f q3: %f\r\n", q0, q1, q2, q3);

		vTaskDelay(250);

	}
}
