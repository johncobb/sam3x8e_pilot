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

static uint8_t delay_factor = 0;
static uint16_t ms_delay = 0;
static uint16_t speed = 0;
static uint16_t steps_perrevolution = 0;


void stepper_init(uint8_t ms1, uint8_t ms2, uint8_t step, uint8_t sleep, uint8_t dir, uint16_t rpm, uint16_t steps_per_rev)
{
	io_ms1 = ms1;
	io_ms2 = ms2;
	io_step = step;
	io_sleep = sleep;
	io_dir = dir;

	clockwise = true;
	step_mode = STEP_FULL;
	stepper_setstepsperrevolution(steps_per_rev);
	stepper_setspeed(rpm);
	stepper_wake();

}

void stepper_tick(void)
{

}

void stepper_sleep(void)
{
	asleep = true;
	pio_set_pin_low(PIN_SLEEP); 	// LOW
}

void stepper_wake(void)
{
	asleep = false;
	pio_set_pin_high(PIN_SLEEP); 	// HIGH
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

uint16_t stepper_getspeed(void)
{
	return speed;
}

uint16_t stepper_getstepsperrevolution(void)
{
	return steps_perrevolution;
}

void stepper_setmode(uint8_t mode)
{
	step_mode = mode;

	switch(step_mode) {
		case STEP_FULL:
			// set io_ms1 LOW
			// set io_ms2 LOW
			delay_factor = 1;
			break;

		case STEP_HALF:
			// set io_ms1 HIGH
			// set io_ms2 LOW
			delay_factor = 2;
			break;

		case STEP_QUARTER:
			// set io_ms1 LOW
			// set io_ms2 HIGH
			delay_factor = 4;
			break;

		case STEP_EIGHT:
			// set io_ms1 HIGH
			// set io_ms2 HIGH
			delay_factor = 8;
			break;
	}
}

void stepper_setspeed(uint16_t rpm)
{
	speed = rpm;
	uint16_t delay_per_sec = (60/rpm)/steps_perrevolution;
	ms_delay = (uint32_t)(delay_per_sec * 1000);

}

void stepper_step(uint16_t number_of_steps)
{
	uint8_t sleep_delay = ms_delay/delay_factor;

	if(number_of_steps >= 0) {
		if(clockwise)
			pio_set_pin_low(PIN_DIR); 	// LOW
		else
			pio_set_pin_high(PIN_DIR); 	// HIGH

		for (uint8_t i=0; i<number_of_steps; i++) {
			pio_set_pin_low(PIN_STEP); 	// LOW
			pio_set_pin_high(PIN_STEP); // HIGH
			// delay_ms(sleep_delay);
			vTaskDelay(sleep_delay);
		}
	} else { // going in reverse (number_of_steps is negative)
		if(clockwise)
			pio_set_pin_high(PIN_DIR); 	// HIGH
		else
			pio_set_pin_low(PIN_DIR); 	// LOW

		for (uint8_t i=number_of_steps; i<=0; i++) {
			pio_set_pin_low(PIN_STEP); 	// LOW
			pio_set_pin_high(PIN_STEP);	// HIGH
			// delay_ms(sleep_delay);
			vTaskDelay(sleep_delay);
		}

	}
}

void stepper_rotate(uint16_t degrees)
{
	uint16_t degrees_per_step = 360/ stepper_getstepsperrevolution();
	uint16_t number_of_steps = degrees/degrees_per_step;

	stepper_step(number_of_steps * delay_factor);

}





