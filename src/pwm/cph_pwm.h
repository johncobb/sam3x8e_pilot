/*
 * cph_pwm.h
 *
 *  Created on: Jan 11, 2016
 *      Author: jcobb
 */

#ifndef SRC_PWM_CPH_PWM_H_
#define SRC_PWM_CPH_PWM_H_

#include <cph.h>



/** PWM frequency in Hz */
#define PWM_FREQUENCY  50
/** PWM period value */
#define PERIOD_VALUE   50
/** Initial duty cycle value */
#define INIT_DUTY_VALUE  0
/** Initial dead time value */
#define INIT_DEAD_TIME   5
/** Maximum synchronous update period */
#define MAX_SYNC_UPDATE_PERIOD  PWM_SCUP_UPR_Msk

/** Duty cycle buffer length for three channels */
#define DUTY_BUFFER_LENGTH      ((PERIOD_VALUE - INIT_DUTY_VALUE + 1) * 3)

pwm_channel_t g_pwm_channel_led;
extern uint32_t ul_duty; /* PWM duty cycle rate */
extern uint8_t fade_in;  /* LED fade in flag */

void cph_pwm_init(void);
void cph_pwm_setdutycycle(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_duty);
#endif /* SRC_PWM_CPH_PWM_H_ */
