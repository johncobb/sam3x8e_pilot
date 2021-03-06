
#ifndef _CPH_BOARD_H
#define _CPH_BOARD_H

#include "compiler.h"
#include "system_sam3x.h"
#include "exceptions.h"
#include "pio.h"
#include "pmc.h"


#define UNIT_TEST_YIELD		vTaskDelay(10); continue;

// special function to initialize usart after configuring pins
void board_init_modem_usart(void);
void board_init_wan_usart(void);

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)


extern volatile uint32_t g_ul_ms_ticks;

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US   15625

/** Name of the board */
#define BOARD_NAME "CPH-BOARD"
/** Board definition */
#define cphBoard
/** Family definition (already defined) */
#define sam3x
/** Core definition */
#define cortexm3


/*! PWM channel for LED0 */
#define CHANNEL_PWM_LED0 PWM_CHANNEL_4

/*! PWM "PWM7" LED0 pin definitions.*/
#define PIN_PWM_LED0_GPIO    PIO_PC21_IDX
#define PIN_PWM_LED0_FLAGS   (PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_PWM_LED0_CHANNEL PWM_CHANNEL_4


#define PWM_LED0		PWM
#define PWM_LED0_ID 	ID_PWM

#define PINS_PWM_PIO	PIOC
#define PINS_PWM_ID		ID_PIOC
#define PINS_PWM_TYPE	PIO_PERIPH_B
#define PINS_PWM_MASK	PIO_PC21
#define PINS_PWM_ATTR	PIO_DEFAULT


/* ------------------------------------------------------------------------ */
/* UART                                                                     */
/* ------------------------------------------------------------------------ */
/*! UART pins (UTXD0 and URXD0) definitions, PA8,9. (labeled RX0->0 and TX0->1)*/


#define PRINTF_USART		UART
#define PRINTF_USART_ID		ID_UART

#define PINS_UART_PIO  PIOA
#define PINS_UART_ID   ID_PIOA
#define PINS_UART_TYPE PIO_PERIPH_A
#define PINS_UART_MASK (PIO_PA8A_URXD | PIO_PA9A_UTXD)
#define PINS_UART_ATTR PIO_DEFAULT


// MODEM USART0
/* ------------------------------------------------------------------------ */
/* USART0                                                                   */
/* ------------------------------------------------------------------------ */
/*! USART0 pin RX  (labeled RX1 19)*/
/*! USART0 pin TX  (labeled TX1 18) */
/*! USART0 pin CTS  (labeled 24) */
/*! USART0 pin RTS  (labeled 2) */

#define MODEM_USART			USART0
#define MODEM_USART_ID		ID_USART0



#define PINS_USART0_PIO		PIOA
#define PINS_USART0_ID		ID_PIOA
#define PINS_USART0_TYPE	PIO_PERIPH_A
#define PINS_USART0_MASK 	(PIO_PA10A_RXD0 | PIO_PA11A_TXD0 | PIO_PB26A_CTS0 | PIO_PB25A_RTS0)
#define PINS_USART0_ATTR	PIO_DEFAULT


// Invensense IMU TWI interface

extern void imu_process_interrupt(uint32_t id, uint32_t mask);


#define IMU_TWI		TWI1
#define IMU_TWI_ID	ID_TWI1

#define IMU_IRQ_PIO					PIOC
#define IMU_IRQ_PIO_ID				ID_PIOC
#define IMU_IRQ_IDX					PIO_PC23_IDX
#define IMU_IRQ_MASK				PIO_PC23
#define IMU_IRQ_IRQ					PIOC_IRQn


#define IMU_IRQ_TYPE				PIO_INPUT
#define IMU_IRQ_ATTR				(PIO_IT_RISE_EDGE | PIO_DEFAULT)
#define IMU_IRQ_FLAGS				(IMU_IRQ_TYPE | IMU_IRQ_ATTR)


// WAN USART1
/* ------------------------------------------------------------------------ */
/* USART1                                                                   */
/* ------------------------------------------------------------------------ */
/*! USART1 pin RX  (labeled RX2 17)*/
/*! USART1 pin TX  (labeled TX2 16) */
/*! USART1 pin CTS  (labeled ??) */
/*! USART1 pin RTS  (labeled ??) */
#define WAN_USART			USART1
#define WAN_USART_ID		ID_USART1

#define PINS_USART1_PIO		PIOA
#define PINS_USART1_ID		ID_PIOA
#define PINS_USART1_TYPE	PIO_PERIPH_A
#define PINS_USART1_MASK 	(PIO_PA12A_RXD1 | PIO_PA13A_TXD1)// | PIO_PA15A_CTS1 | PIO_PA14A_RTS1)
#define PINS_USART1_ATTR	PIO_DEFAULT




// CONSOLE printf USART3

/* ------------------------------------------------------------------------ */
/* USART3                                                                   */
/* ------------------------------------------------------------------------ */
/*! USART3 pin RX  (labeled RX3 15)*/
/*! USART3 pin TX  (labeled TX3 14) */
/*! USART3 pin CTS  (labeled ??) */
/*! USART3 pin RTS  (labeled ??) */
//#define PRINTF_USART		USART3
//#define PRINTF_USART_ID		ID_USART3

#define PINS_USART3_PIO		PIOD
#define PINS_USART3_ID		ID_PIOD
#define PINS_USART3_TYPE	PIO_PERIPH_B
#define PINS_USART3_MASK 	(PIO_PD5B_RXD3 | PIO_PD4B_TXD3)// | PIO_PA15A_CTS1 | PIO_PA14A_RTS1)
#define PINS_USART3_ATTR	PIO_DEFAULT





/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

//#define LED_STATUS_IDX		PIO_PB27_IDX
//
//#define PINS_LED0_PIO			PIOB
//#define PINS_LED0_TYPE		PIO_OUTPUT_0
//#define PINS_LED0_MASK		PIO_PB27
//#define PINS_LED0_ATTR		PIO_DEFAULT


// stepper motor controller functions

// pin 51
#define PIN_STEPPERDIR_IDX		PIO_PC12_IDX
#define PIN_STEPPERDIR_MASK  	(1 << 12)
#define PIN_STEPPERDIR_PIO   	PIOC
#define PIN_STEPPERDIR_ID    	ID_PIOC
#define PIN_STEPPERDIR_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERDIR_ATTR  	PIO_DEFAULT

// pin 50
#define PIN_STEPPERSTEP_IDX		PIO_PC13_IDX
#define PIN_STEPPERSTEP_MASK  	(1 << 13)
#define PIN_STEPPERSTEP_PIO   	PIOC
#define PIN_STEPPERSTEP_ID    	ID_PIOC
#define PIN_STEPPERSTEP_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERSTEP_ATTR  	PIO_DEFAULT

// pin 49
#define PIN_STEPPERSLEEP_IDX	PIO_PC14_IDX
#define PIN_STEPPERSLEEP_MASK  	(1 << 14)
#define PIN_STEPPERSLEEP_PIO   	PIOC
#define PIN_STEPPERSLEEP_ID    	ID_PIOC
#define PIN_STEPPERSLEEP_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERSLEEP_ATTR  	PIO_DEFAULT

// pin 48
#define PIN_STEPPERMS1_IDX		PIO_PC15_IDX
#define PIN_STEPPERMS1_MASK  	(1 << 15)
#define PIN_STEPPERMS1_PIO   	PIOC
#define PIN_STEPPERMS1_ID    	ID_PIOC
#define PIN_STEPPERMS1_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERMS1_ATTR  	PIO_DEFAULT

// pin 47
#define PIN_STEPPERMS2_IDX		PIO_PC16_IDX
#define PIN_STEPPERMS2_MASK  	(1 << 16)
#define PIN_STEPPERMS2_PIO   	PIOC
#define PIN_STEPPERMS2_ID    	ID_PIOC
#define PIN_STEPPERMS2_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERMS2_ATTR  	PIO_DEFAULT

// pin 46
#define PIN_STEPPERENABLE_IDX		PIO_PC17_IDX
#define PIN_STEPPERENABLE_MASK  	(1 << 17)
#define PIN_STEPPERENABLE_PIO   	PIOC
#define PIN_STEPPERENABLE_ID    	ID_PIOC
#define PIN_STEPPERENABLE_TYPE  	PIO_OUTPUT_0
#define PIN_STEPPERENABLE_ATTR  	PIO_DEFAULT






/*! LED #0 "L" pin definition (ORANGE).*/
#define PIN_LED_0_IDX	PIO_PB27_IDX
#define PIN_LED_0       {1 << 27, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_LED_0_MASK  (1 << 27)
#define PIN_LED_0_PIO   PIOB
#define PIN_LED_0_ID    ID_PIOB
#define PIN_LED_0_TYPE  PIO_OUTPUT_0
#define PIN_LED_0_ATTR  PIO_DEFAULT



/* Arduino DUE Pin  6*/
#define PIN_LED1       		PIO_PC24_IDX
#define PIN_LED1_MASK  		(1 << 24)
#define PIN_LED1_PIO   		PIOC
#define PIN_LED1_ID    		ID_PIOC
#define PIN_LED1_TYPE  		PIO_OUTPUT_0
#define PIN_LED1_ATTR  		PIO_DEFAULT

/* Arduino DUE Pin  7*/
#define PIN_LED2       		PIO_PC23_IDX
#define PIN_LED2_MASK  		(1 << 23)
#define PIN_LED2_PIO   		PIOC
#define PIN_LED2_ID    		ID_PIOC
#define PIN_LED2_TYPE  		PIO_OUTPUT_0
#define PIN_LED2_ATTR  		PIO_DEFAULT



/*! C.22 WAN_INT_3.*/
#define WAN_INT3_IDX       	PIO_PC22_IDX
#define WAN_INT3_MASK  		(1 << 22)
#define WAN_INT3_PIO   		PIOC
#define WAN_INT3_ID    		ID_PIOC
#define WAN_INT3_TYPE  		PIO_TYPE_PIO_INPUT
#define WAN_INT3_ATTR  		PIO_DEFAULT

/*! C.21 WAN_INT_2.*/
#define WAN_INT2_IDX       	PIO_PC21_IDX
#define WAN_INT2_MASK  		(1 << 21)
#define WAN_INT2_PIO   		PIOC
#define WAN_INT2_ID    		ID_PIOC
#define WAN_INT2_TYPE  		PIO_TYPE_PIO_INPUT
#define WAN_INT2_ATTR  		PIO_DEFAULT

/*! C.29 WAN_INT_1.*/
#define WAN_INT1_IDX       	PIO_PC29_IDX
#define WAN_INT1_MASK  		(1 << 29)
#define WAN_INT1_PIO   		PIOC
#define WAN_INT1_ID    		ID_PIOC
#define WAN_INT1_TYPE  		PIO_TYPE_PIO_INPUT
#define WAN_INT1_ATTR  		PIO_DEFAULT





/*! B.25 MODEM ON/OFF.*/
#define MDM_ONOFF_IDX       	PIO_PB25_IDX
#define MDM_ONOFF_MASK  		(1 << 25)
#define MDM_ONOFF_PIO   		PIOB
#define MDM_ONOFF_ID    		ID_PIOB
#define MDM_ONOFF_TYPE  		PIO_OUTPUT_0
#define MDM_ONOFF_ATTR  		PIO_DEFAULT

/*! C.28 MODEM ENABLE.*/
#define MDM_ENABLE_IDX       	PIO_PC28_IDX
#define MDM_ENABLE_MASK  		(1 << 28)
#define MDM_ENABLE_PIO   		PIOC
#define MDM_ENABLE_ID    		ID_PIOC
#define MDM_ENABLE_TYPE  		PIO_OUTPUT_0
#define MDM_ENABLE_ATTR  		PIO_DEFAULT
//#define MDM_ENABLE_ATTR  		PIO_PULLUP


/*! C.26 MODEM RESET.*/
#define MDM_RESET_IDX       	PIO_PC26_IDX
#define MDM_RESET_MASK  		(1 << 26)
#define MDM_RESET_PIO   		PIOC
#define MDM_RESET_ID    		ID_PIOC
#define MDM_RESET_TYPE  		PIO_OUTPUT_0
#define MDM_RESET_ATTR  		PIO_DEFAULT


/*! C.25 MODEM POWMON.*/
#define MDM_POWMON_IDX       	PIO_PC25_IDX
#define MDM_POWMON_MASK  		(1 << 25)
#define MDM_POWMON_PIO   		PIOC
#define MDM_POWMON_ID    		ID_PIOC
#define MDM_POWMON_TYPE  		PIO_INPUT
#define MDM_POWMON_ATTR			PIO_PULLUP

/*! TWI0 pins definition */
//#define TWI0_DATA_GPIO   PIO_PA17_IDX
//#define TWI0_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
//#define TWI0_CLK_GPIO    PIO_PA18_IDX
//#define TWI0_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

/*! TWI1 pins definition */
//#define TWI1_DATA_GPIO   PIO_PB12_IDX
//#define TWI1_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
//#define TWI1_CLK_GPIO    PIO_PB13_IDX
//#define TWI1_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)


#define PINS_TWI1_PIO		PIOB
#define PINS_TWI1_ID		ID_PIOB
#define PINS_TWI1_TYPE		PIO_PERIPH_A
#define PINS_TWI1_MASK 		(PIO_PB13A_TWCK1 | PIO_PB12A_TWD1)
#define PINS_TWI1_ATTR		PIO_DEFAULT







#endif  // _CPH_BOARD_H
