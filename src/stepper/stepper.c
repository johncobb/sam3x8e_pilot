/*
 * stepper.c
 *
 *  Created on: Jan 7, 2016
 *      Author: jcobb
 */


#include "stepper.h"

// TODO: future pins to be used
//static uint8_t io_enable = 0;
//static uint8_t io_fault = 0;

static uint8_t io_ms1 = 0;
static uint8_t io_ms2 = 0;
static uint8_t io_step = 0;
static uint8_t io_sleep = 0;
static uint8_t io_dir = 0;
static uint8_t step_mode = STEP_FULL;

static bool clockwise = true;
static bool asleep = true;
static bool enable = true;
static bool reset = true;

static uint8_t delay_factor = 0;
static uint16_t step_delay_ms = 0;
static float speed = 0;
static float steps_perrevolution = 0;


void stepper_init(uint8_t ms1, uint8_t ms2, uint8_t step, uint8_t sleep, uint8_t dir, float rpm, uint16_t steps_per_rev)
{
	io_ms1 = ms1;
	io_ms2 = ms2;
	io_step = step;
	io_sleep = sleep;
	io_dir = dir;

	clockwise = true;
	stepper_setmode(STEP_FULL);

	stepper_setstepsperrevolution(steps_per_rev);
	stepper_setspeed(rpm);
	stepper_wake();

}

void stepper_tick(void)
{

}

void stepper_enalbe(bool flag)
{
	enable = flag;
	if(enable)
		pio_set_pin_high(PIN_STEPPERENABLE_IDX); // HIGH
	else
		pio_set_pin_low(PIN_STEPPERENABLE_IDX); // LOW

}

void stepper_reset(bool reset)
{

}

void stepper_sleep(void)
{
	asleep = true;
	pio_set_pin_low(PIN_STEPPERSLEEP_IDX); 	// LOW
}

void stepper_wake(void)
{
	asleep = false;
	pio_set_pin_high(PIN_STEPPERSLEEP_IDX); // HIGH
}

void stepper_setstepsperrevolution(uint16_t steps)
{
	steps_perrevolution = steps;
}

void stepper_reverse(void)
{
	clockwise = !clockwise;
}

uint8_t stepper_getstepmode(void)
{
	return step_mode;
}

float stepper_getspeed(void)
{
	return speed;
}

float stepper_getstepsperrevolution(void)
{
	return steps_perrevolution;
}

void stepper_setmode(uint8_t mode)
{
	step_mode = mode;

	switch(step_mode) {
		case STEP_FULL:
			pio_set_pin_low(PIN_STEPPERMS1_IDX);
			pio_set_pin_low(PIN_STEPPERMS2_IDX);
			// set io_ms1 LOW
			// set io_ms2 LOW
			delay_factor = 1;
			break;

		case STEP_HALF:
			pio_set_pin_high(PIN_STEPPERMS1_IDX);
			pio_set_pin_low(PIN_STEPPERMS2_IDX);
			// set io_ms1 HIGH
			// set io_ms2 LOW
			delay_factor = 2;
			break;

		case STEP_QUARTER:
			pio_set_pin_low(PIN_STEPPERMS1_IDX);
			pio_set_pin_high(PIN_STEPPERMS2_IDX);
			// set io_ms1 LOW
			// set io_ms2 HIGH
			delay_factor = 4;
			break;

		case STEP_EIGHT:
			pio_set_pin_high(PIN_STEPPERMS1_IDX);
			pio_set_pin_high(PIN_STEPPERMS2_IDX);
			// set io_ms1 HIGH
			// set io_ms2 HIGH
			delay_factor = 8;
			break;
	}
}

void stepper_setspeed(float rpm)
{

//	speed = rpm;
//	float delay_per_sec = (60.0f/rpm)/steps_perrevolution;
//	step_delay_ms = (uint16_t)(delay_per_sec * 1000 *1000);

	// set speed in revs per min.
	speed = rpm;
	// desired rotations per min. * 1000 div
	step_delay_ms = (60 * 1000/steps_perrevolution/rpm);
}

void stepper_step(int16_t number_of_steps)
{

	uint16_t sleep_delay = step_delay_ms/delay_factor;

	printf("stepper_step: steps=%d clockwise=%d delay=%d\r\n", number_of_steps, clockwise, sleep_delay);

	if(number_of_steps >= 0) {
		if(clockwise)
			pio_set_pin_low(PIN_STEPPERDIR_IDX); 	// LOW
		else
			pio_set_pin_high(PIN_STEPPERDIR_IDX); 	// HIGH

		for (int i=0; i<number_of_steps; i++) {
			printf("current_step=%d\r\n", i);
			pio_set_pin_low(PIN_STEPPERSTEP_IDX); 	// LOW
			pio_set_pin_high(PIN_STEPPERSTEP_IDX); // HIGH
			vTaskDelay(sleep_delay);
		}
	} else { // going in reverse (number_of_steps is negative)
		if(clockwise)
			pio_set_pin_high(PIN_STEPPERDIR_IDX); 	// HIGH
		else
			pio_set_pin_low(PIN_STEPPERDIR_IDX); 	// LOW

		for (int i=number_of_steps; i<=0; i++) {
			printf("current_step=%d\r\n", i);
			pio_set_pin_low(PIN_STEPPERSTEP_IDX); 	// LOW
			pio_set_pin_high(PIN_STEPPERSTEP_IDX);	// HIGH
			vTaskDelay(sleep_delay);
		}
	}
}

void stepper_rotate(int16_t degrees)
{
	float degrees_per_step = 360.0f / stepper_getstepsperrevolution();
	int16_t number_of_steps = degrees/degrees_per_step;

	stepper_step(number_of_steps * delay_factor);

}





