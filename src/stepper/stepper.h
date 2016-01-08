/*
 * stepper.h
 *
 *  Created on: Jan 7, 2016
 *      Author: jcobb
 */

#ifndef SRC_STEPPER_STEPPER_H_
#define SRC_STEPPER_STEPPER_H_

#include <cph.h>

typedef enum STEP_MODE {
	STEP_FULL = 0,
	STEP_HALF,
	STEP_QUARTER,
	STEP_EIGHT
}step_mode_t;

#define STEPPER_STEPS_PER_REV	200
#define	STEPPER_RPM				60.0

#define PIN_DIR 		1
#define PIN_STEP 		2
#define PIN_SLEEP 		3
#define PIN_MS1			4
#define PIN_MS2			5


void stepper_init(uint8_t ms1, uint8_t ms2, uint8_t step, uint8_t sleep, uint8_t dir, float rpm, uint16_t steps_per_rev);
void stepper_tick(void);
void stepper_enalbe(bool flag);
void stepper_reset(bool reset);
void stepper_sleep(void);
void stepper_wake(void);
void stepper_step(int16_t number_of_steps);
void stepper_rotate(int16_t degress);
void stepper_reverse(void);
void stepper_setmode(uint8_t mode);
void stepper_setspeed(float speed);
void stepper_setstepsperrevolution(uint16_t steps);

bool is_awake(void);
uint8_t stepper_getstepmode(void);
float stepper_getspeed(void);
float stepper_getstepsperrevolution(void);




#endif /* SRC_STEPPER_STEPPER_H_ */
