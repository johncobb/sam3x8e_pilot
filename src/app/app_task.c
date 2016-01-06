/*
 * app_task.c
 *
 *  Created on: Jul 22, 2015
 *      Author: jcobb
 */

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <cph.h>
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_usart_serial.h"
#include "sysclk.h"
#include "tcpip.h"
#include "app_task.h"
#include "gimbal.h"

xSemaphoreHandle app_start_signal = 0;

static volatile bool start_task = false;
static void app_handler_task(void *pvParameters);
static void balance(void);
static void calc_pid(void);



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


static void app_handler_task(void *pvParameters)
{
	gimbal_init();

	while(true) {

		gimbal_tick();
		calc_pid();
		vTaskDelay(10);

	}
}

void balance(void)
{

}

static float k_p = 1.0f;
static float k_i = .05f;
static float k_d = .25f;

static float center_val = 0.0f;
static float integral = 0.0f;
static float last_error = 0.0f;
static float pid_val_pitch = 0.0f;
static float pid_val_roll = 0.0f;
static float pid_val_yaw = 0.0f;

static uint32_t t_now = 0;
static uint32_t t_lastread = 0;

static uint32_t pid_printf_timeout = 0;

void calc_pid(void)
{

	uint32_t t_now = cph_get_millis();

	float delta_t = (t_now-t_lastread)/1000.0f;

	float pid_error = (center_val - roll);
	float p_term = k_p * pid_error;
	integral += pid_error * delta_t;
	integral = constrain(integral, -1.0f, 1.0f); // limit the error
//	float i_term = (k_i * 100.0f) * integral;
//	float d_term = (k_d * 100.0f) * (pid_error - last_error)/delta_t;

	float i_term = k_i * integral;
	float d_term = k_d * (pid_error - last_error)/delta_t;
	last_error = pid_error;

	pid_val_pitch = p_term + i_term + d_term;

	t_lastread = t_now;


	if(cph_get_millis() > pid_printf_timeout) {
		printf("pid %f err %f int %f dlt %f\r\n", pid_val_pitch, pid_error, integral, delta_t);
		pid_printf_timeout = cph_get_millis() + 100;
	}

}





