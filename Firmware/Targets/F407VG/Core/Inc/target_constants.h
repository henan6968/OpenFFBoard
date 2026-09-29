/*
 * target_constants.h
 *
 *  Created on: 06.11.2020
 *      Author: Yannick
 */

#ifndef INC_TARGET_CONSTANTS_H_
#define INC_TARGET_CONSTANTS_H_

/*
 * Add settings and peripheral maps for this specific target here
 */

// Hardware name string
#define HW_TYPE "F407VG"
#define HW_TYPE_INT 2
#define FW_DEVID 0x413 // F407

#include "main.h"

// Enabled features
#define DEFAULTMAIN 1 // FFBWheel
// Main classes
#define FFBWHEEL
#define FFBJOYSTICK
#define MIDI
#define TMCDEBUG
#define CANBRIDGE
#define FFBHIDEXT
#define RMDCAN
#define CANINPUTMAIN

/*
 * FFBWheel uses 2 FFB axis descriptor instead of 1 axis.
 * Might improve compatibility with direct input but will report a 2 axis ffb compatible device
 */
//#define FFBWHEEL_USE_1AXIS_DESC

// Extra features
#define LOCALBUTTONS
#define SPIBUTTONS
#define SHIFTERBUTTONS
#define PCF8574BUTTONS // Requires I2C
#define ANALOGAXES
#define TMC4671DRIVER
#define PWMDRIVER
#define LOCALENCODER
#define CANBUS
#define ODRIVE
#define VESC
#define MTENCODERSPI // requires SPI3
#define CANBUTTONS // Requires CAN
#define CANANALOG // Requires CAN
#define BISSENCODER // Requires SPI3
#define SSIENCODER // Requires SPI3
#define ADS111XANALOG // Requires I2C
#define UARTCOMMANDS
#define SIMPLEMOTION // Requires motor gpio pin
#define VESC_UART // VESC over UART on motor_uart, see VESC_UART_ALT_PORT below

//----------------------
#define BTNFAILSAFE // Use user button to force board into failsafe mainclass

#define TIM_ENC htim3
// Timer 3 is used by the encoder.
#define TIM_PWM htim1

#define TIM_MICROS_HALTICK htim7 // Micros timer MUST be reset by hal tick timer or isr to count microseconds since last tick
#define TIM_USER htim9 // Timer with full core clock speed available for the mainclass
#define TIM_TMC htim6 // Timer running at half clock speed
#define TIM_TMC_BCLK SystemCoreClock / 2
#define TIM_TMC_ARR 250 // 4khz
#define TIM_FFB htim13

extern UART_HandleTypeDef huart1;
#define UART_PORT_EXT huart1 // main uart port

extern UART_HandleTypeDef huart3;
#define UART_PORT_MOTOR huart3 // motor uart port

// ---------------------------------------------------------------------------
// VESC over UART: which F407 UART is wired to the VESC
//
// Default (VESC_UART_ALT_PORT undefined):
//     motor_uart = USART3  PB10 = TX / PB11 = RX  -> VESC COMM header
//     VESC speed = appconf.app_uart_baudrate (460800 here)
//
// With VESC_UART_ALT_PORT defined:
//     motor_uart = USART1  PB6  = TX / PB7  = RX  -> VESC "UART2" socket
//     VESC speed = 115200, fixed by HW_UART_P_BAUD
//
// The swap is done here and nowhere else: motor_uart / external_uart are just
// aliases, so VescUART.cpp, MotorSimplemotion.cpp and cpp_target_config.cpp all
// compile unchanged and simply follow.
//
// Two consequences of defining VESC_UART_ALT_PORT:
//   * UARTCOMMANDS must be off - UART_CommandInterface() reserves
//     external_uart (FFBoardMain.h:62) and would otherwise hold USART1, leaving
//     the VESC driver with no port. The USB CDC console covers the same need.
//   * SIMPLEMOTION also follows motor_uart onto USART1. It is unused here; if it
//     is ever needed, give it its own port instead of sharing this one.
// ---------------------------------------------------------------------------
//#define VESC_UART_ALT_PORT

#ifdef VESC_UART_ALT_PORT
	#undef UART_PORT_MOTOR
	#undef UART_PORT_EXT
	#define UART_PORT_MOTOR huart1 // USART1 PB6/PB7 -> VESC UART2 socket
	#define UART_PORT_EXT huart3   // USART3 PB10/PB11, now free
#else
	#define UARTCOMMANDS
#endif

#define SIMPLEMOTION // Requires motor gpio pin
#define VESC_UART // VESC over UART on motor_uart, see VESC_UART_ALT_PORT above

#define UART_BUF_SIZE 1 // How many bytes to expect via DMA

extern I2C_HandleTypeDef hi2c1;
#define I2C_PORT hi2c1



// ADC Channels
#define ADC1_CHANNELS 8 	// how many analog input values to be read by dma
#define ADC2_CHANNELS 2		// VSENSE

extern ADC_HandleTypeDef hadc2;
#define VSENSE_HADC hadc2
#define ADC_CHAN_VINT 1	// adc buffer index of internal voltage sense
#define ADC_CHAN_VEXT 0 // adc buffer index of supply voltage sense
extern volatile uint32_t ADC2_BUF[ADC2_CHANNELS]; // Buffer
#define VSENSE_ADC_BUF ADC2_BUF

extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS]; // Buffer
#define TEMPSENSOR_ADC_VAL ADC1_BUF[6] // ADC1 ch 7
#define ADC_INTREF_VAL ADC1_BUF[7] // ADC1 ch 8.

extern ADC_HandleTypeDef hadc1;
#define AIN_HADC hadc1	// main adc for analog pins
#define ADC_PINS 6	// Amount of analog channel pins
#define ADC_CHAN_FPIN 0 // First analog channel pin. last channel = fpin+ADC_PINS-1
//#define VOLTAGE_MULT_DEFAULT 30.12 // mV adc * scaler = voltage

#define BUTTON_PINS 8

extern SPI_HandleTypeDef hspi1;
#define HSPIDRV hspi1
extern SPI_HandleTypeDef hspi2;
#define HSPI2 hspi2
extern SPI_HandleTypeDef hspi3;
#define EXT3_SPI_PORT hspi3

// CAN
#ifdef CANBUS
extern CAN_HandleTypeDef hcan1;
#define CANPORT hcan1
#endif

#define DEBUGPIN // GP1 pin. see cpp target constants

//Flash. 2 pages used
/* EEPROM start address in Flash
 * PAGE_ID sectors 1 and 2!
 * */
#define USE_EEPROM_EMULATION
#define PAGE0_ID               FLASH_SECTOR_1
#define PAGE1_ID               FLASH_SECTOR_2
#define EEPROM_START_ADDRESS  ((uint32_t)0x08004000) /* EEPROM emulation start address: from sector1*/
#define PAGE_SIZE             (uint32_t)0x4000  /* Page size = 16KByte */


// System
// BKPSRAM positions
#define DFU_JUMP_MAGIC_ADR BKPSRAM_BASE + 0

#define CCRAM_SEC ".ccmram"

#define SIGBNATUREBASEADR 0 // First block in OTP

#endif /* INC_TARGET_CONSTANTS_H_ */
