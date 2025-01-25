/**
 * RP2040 FreeRTOS Template
 *
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.2.0
 * @license   MIT
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_


// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
// C
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <ctype.h>
#include <time.h>
 /*
 // c++
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <cstdint>
 */ 

// Pico SDK
#include "pico/stdlib.h"            // Includes `hardware_gpio.h`
#include "pico/binary_info.h"
#include "hardware/i2c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * CONSTANTS
 * 
 */
 
#define         PAUSE_TIMER_ID    		0
#define         DOT_TIMER_ID            0
#define         ADC_TIMER_ID            0

#define			D0_PIN		           0  // pin 1
#define			D1_PIN			       1  // pin 2
#define			D2_PIN		           2  // pin 4
#define			D3_PIN		           3  // pin 5
#define			D4_PIN			       4  // pin 6
#define			D5_PIN			       5  // pin 7
#define			D6_PIN			       6  // pin 9
#define			D7_PIN			       7  // pin 10
#define			D8_PIN		           8  // pin 11
#define			D9_PIN		           9  // pin 12
#define			D10_PIN			      10  // pin 14
#define			D11_PIN			      11  // pin 15
#define			D12_PIN		          12  // pin 16
#define			D13_PIN		          13  // pin 17
#define			D14_PIN			      14  // pin 19
#define			D15_PIN			      15  // pin 20
#define			D16_PIN			      16  // pin 21
#define			D17_PIN			      17  // pin 22
#define			D18_PIN		          18  // pin 24
#define			D19_PIN		          19  // pin 25
#define			D20_PIN		          20  // pin 26
#define			D21_PIN			      21  // pin 27
#define			D22_PIN		          22  // pin 29
#define			D26_PIN	          	  26  // pin 31
#define			D27_PIN	              27  // pin 32
#define			D28_PIN	              28  // pin 34
#define			ADC0	          	  26  // pin 31
#define			ADC1                  27  // pin 32
#define			ADC2	              28  // pin 34
#define			SW4 				   4
#define			SW5 				   5
#define			SW6 				   6
#define			SW7 				   7
#define			A3_PIN	              29
#define         Feather_LED_PIN       13
#define         PICO_LED_PIN          25
#define			D14			      	  14
#define			D15			      	  15
#define			G16			      	  16

// Set a delay time
const TickType_t ms_delay5  =    5 / portTICK_PERIOD_MS;
const TickType_t ms_delay10 =   10 / portTICK_PERIOD_MS;
const TickType_t ms_delay20 =   20 / portTICK_PERIOD_MS;
const TickType_t ms_delay30 =   30 / portTICK_PERIOD_MS;
const TickType_t ms_delay50 =   50 / portTICK_PERIOD_MS;
const TickType_t ms_delay60 =   60 / portTICK_PERIOD_MS;
const TickType_t ms_delay75 =   75 / portTICK_PERIOD_MS;
const TickType_t ms_delay100 = 100 / portTICK_PERIOD_MS;
const TickType_t ms_delay150 = 150 / portTICK_PERIOD_MS;
const TickType_t ms_delay200 = 200 / portTICK_PERIOD_MS;
const TickType_t ms_delay300 = 300 / portTICK_PERIOD_MS;
const TickType_t ms_delay400 = 400 / portTICK_PERIOD_MS;
const TickType_t ms_delay500 = 500 / portTICK_PERIOD_MS;
const TickType_t ms_delay600 = 600 / portTICK_PERIOD_MS;
const TickType_t ms_delay700 = 700 / portTICK_PERIOD_MS;
const TickType_t ms_delay173 = 173 / portTICK_PERIOD_MS;
const TickType_t ms_delay237 = 237 / portTICK_PERIOD_MS;
const TickType_t ms_delay317 = 317 / portTICK_PERIOD_MS;
const TickType_t ms_delay349 = 349 / portTICK_PERIOD_MS;

/**
 * PROTOTYPES
 */
 
void print_msg(const char* msg);
void log_device_info(void);
void configure_gpio(void);
char *get_string(const char *prompt);
void cw_timer_fired_callback(TimerHandle_t timer);

#ifdef __cplusplus
}           // extern "C"
#endif


#endif      // _MAIN_H_
