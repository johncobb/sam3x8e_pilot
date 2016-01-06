/**
 * \file
 *
 * \brief Arduino Due/X board init.
 *
 * Copyright (c) 2011 - 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */





#include "board.h"
#include "conf_board.h"
#include "pio_handler.h"



void board_init(void)
{
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#endif


#ifdef CONF_BOARD_MODEM_CONTROL

//	pio_configure(MDM_ONOFF_PIO, MDM_ONOFF_TYPE, MDM_ONOFF_MASK, MDM_ONOFF_ATTR);
//	pio_configure(MDM_ENABLE_PIO, MDM_ENABLE_TYPE, MDM_ENABLE_MASK, MDM_ENABLE_ATTR);
//	pio_configure(MDM_RESET_PIO, MDM_RESET_TYPE, MDM_RESET_MASK, MDM_RESET_ATTR);
//
//	pmc_enable_periph_clk(ID_PIOC);
//	pio_configure(MDM_POWMON_PIO, MDM_POWMON_TYPE, MDM_POWMON_MASK, MDM_POWMON_ATTR);


#endif

#ifdef CONF_BOARD_WAN_CONTROL

//	pio_configure(WAN_INT1_PIO, WAN_INT1_TYPE, WAN_INT1_MASK, WAN_INT1_ATTR);
//	pio_configure(WAN_INT2_PIO, WAN_INT2_TYPE, WAN_INT2_MASK, WAN_INT2_ATTR);
//	pio_configure(WAN_INT3_PIO, WAN_INT3_TYPE, WAN_INT3_MASK, WAN_INT3_ATTR);
//	// enable the wan peripheral clock
//	pmc_enable_periph_clk(ID_PIOA);
#endif


#ifdef CONF_BOARD_LEDS
	//pio_configure(PINS_LED0_PIO, PINS_LED0_TYPE, PINS_LED0_MASK, PINS_LED0_ATTR);
	pio_configure(PIN_LED_0_PIO, PIN_LED_0_TYPE, PIN_LED_0_MASK, PIN_LED_0_ATTR);
	pio_configure(PIN_LED1_PIO, PIN_LED1_TYPE, PIN_LED1_MASK, PIN_LED1_ATTR);
	pio_configure(PIN_LED2_PIO, PIN_LED2_TYPE, PIN_LED2_MASK, PIN_LED2_ATTR);
#endif


	pmc_enable_periph_clk(ID_PIOA);

#ifdef CONF_BOARD_TWI1

// 20/21 - TWI1
//	  { PIOB, PIO_PB12A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD1 - SDA0
//	  { PIOB, PIO_PB13A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK1 - SCL0

// 70/71 - TWI0
//	  { PIOA, PIO_PA17A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA1
//	  { PIOA, PIO_PA18A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL1

// 79 - TWI0 all pins
//	  { PIOA, PIO_PA17A_TWD0|PIO_PA18A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

	// 80 - TWI1 all pins
//	  { PIOB, PIO_PB12A_TWD1|PIO_PB13A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },


	pio_configure(PINS_TWI1_PIO, PINS_TWI1_TYPE, PINS_TWI1_MASK, PINS_TWI1_ATTR);

	pmc_enable_periph_clk(ID_PIOB);



#endif

#ifdef CONF_PRINTF_USART
	pio_configure(PINS_UART_PIO, PINS_UART_TYPE, PINS_UART_MASK, PINS_UART_ATTR);

#endif


	pio_configure_pin(IMU_IRQ_IDX, IMU_IRQ_FLAGS);

	pio_set_input(IMU_IRQ_PIO, IMU_IRQ_MASK, PIO_DEFAULT);
	pio_handler_set(IMU_IRQ_PIO, IMU_IRQ_PIO_ID, IMU_IRQ_MASK, IMU_IRQ_ATTR, imu_process_interrupt);
	pio_enable_interrupt(IMU_IRQ_PIO, IMU_IRQ_MASK);

	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 255
	NVIC_SetPriority(PIOC_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);//TODO: Is this correct IRQ priority w/ FreeRTOS?
	NVIC_EnableIRQ(PIOC_IRQn);

	pmc_enable_periph_clk(ID_PIOC);


}


void board_init_modem_usart(void)
{
#ifdef CONF_MODEM_USART
	pio_configure(PINS_USART0_PIO, PINS_USART0_TYPE, PINS_USART0_MASK, PINS_USART0_ATTR);
#endif
}

void board_init_wan_usart(void)
{
#ifdef CONF_WAN_USART
//	pio_configure(PINS_USART1_PIO, PINS_USART1_TYPE, PINS_USART1_MASK, PINS_USART1_ATTR);
#endif
}
