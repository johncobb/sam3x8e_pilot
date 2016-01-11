/*
 * cph_pwm.c
 *
 *  Created on: Jan 11, 2016
 *      Author: jcobb
 */


#include "cph_pwm.h"


/** Duty cycle buffer for PDC transfer */
uint16_t g_us_duty_buffer[DUTY_BUFFER_LENGTH];

/** PDC transfer packet */
pdc_packet_t g_pdc_tx_packet;


/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;




void cph_pwm_init(void)
{
	pwm_channel_t channel;

	pmc_enable_periph_clk(ID_PWM);

	pwm_channel_disable(PWM, PIN_PWM_LED0_CHANNEL);


	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;

	pwm_channel_init(PWM, &g_pwm_channel_led);
	pwm_channel_enable_interrupt(PWM, PIN_PWM_LED0_CHANNEL, 0);


	NVIC_DisableIRQ(PWM_IRQn);
	NVIC_ClearPendingIRQ(PWM_IRQn);
	NVIC_SetPriority(PWM_IRQn, 0);
	NVIC_EnableIRQ(PWM_IRQn);

	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM, PIN_PWM_LED0_CHANNEL);

}

void cph_pwm_setdutycycle(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_duty)
{
	pwm_channel_update_duty(p_pwm, p_channel, ul_duty);
}

void PWM_Handler(void)
{

//	static uint32_t ul_count = 0;  /* PWM counter value */
//	static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */
//	static uint8_t fade_in = 1;  /* LED fade in flag */
//
	uint32_t events = pwm_channel_get_interrupt_status(PWM);


	/* Interrupt on PIN_PWM_LED0_CHANNEL */
//		if ((events & (1 << PIN_PWM_LED0_CHANNEL)) == (1 << PIN_PWM_LED0_CHANNEL)) {
//			ul_count++;
//			/* Fade in/out */
//			if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
//				/* Fade in */
//				if (fade_in) {
//					ul_duty++;
//					if (ul_duty == PERIOD_VALUE) {
//						fade_in = 0;
//					}
//					} else {
//					/* Fade out */
//					ul_duty--;
//					if (ul_duty == INIT_DUTY_VALUE) {
//						fade_in = 1;
//					}
//				}
//
//				/* Set new duty cycle */
//				ul_count = 0;
//				g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
//
//				pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
//			}
//		}

}
