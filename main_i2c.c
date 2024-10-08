   /**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *  cd ~/FreeRTOS-Play/build/App-SW
 * 
 * Simulate Reading measurement points, display ADC value, look for
 * Measurement alarm boundary.  If measurement is less than boundary,
 * blink seven seg display.  If the measured value moves higher than 
 * alarm boundary, continue blinking until blinking is acknowledged.
 *   
 */
#include <stdio.h>
#include "main.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "../Common/Seven_Seg_i2c/seven_seg.h"  
#include "../Common/pcf8575i2c.h"

#define HEX_INTERVALS 0X100
#define DEC_INTERVALS 400
#define MIN_COUNT 3
#define ALARM_BOUNDARY 8
#define HEX_ALARM_BOUNDARY 0X800
     

/*
 * GLOBALS
 */
 
// This is the inter-task queue
volatile QueueHandle_t xQblink = NULL;
volatile QueueHandle_t xQsw4 = NULL;

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;
 
// FROM 1.0.1 Record references to the tasks
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t gpio_led_task_handle = NULL;
TaskHandle_t adc_task_handle = NULL;
TaskHandle_t sw4_debounce_task_handle = NULL;
/*
 * FUNCTIONS
 */
/* 
 * @brief Measure Value of ADC Input 0, 
 * Display value of ADC on a seven segment LED module.
 * Set up as a monitoring display. The 7seg display will blink
 * if the ADC input is less than a pre-selected value.
 */

void adc_task(void* unused_arg) {

    uint32_t now = 1 ;
    uint32_t ack_flag = 0;
    uint32_t measured = 1;
    uint32_t sw4_state = 0;
    uint32_t sw4_buffer = 0;
    uint32_t blink_buffer = 0;
    const uint32_t blanked = 20 ;
   UBaseType_t uxMessagesWaiting  = 0;

    while (true) {
        // Measure ADC
        now = adc_read();
        measured = now/(HEX_INTERVALS);  
        printf("\n ADC output now = %x", now);
//        if(measured < ALARM_BOUNDARY) {      // ** blink drive  **
        if(now < HEX_ALARM_BOUNDARY) {      // ** blink drive  **

            ack_flag = 1;
            uxMessagesWaiting = uxQueueMessagesWaiting(xQblink);
            if(uxQueueMessagesWaiting){  // xQueuePeek does not empty Q
                xQueuePeek(xQblink, &blink_buffer, 0);
            
                if(blink_buffer == 0) show_seven_seg(blanked);
                else show_seven_seg(measured);
                }
            }   // ****  end blink drive  **********
        else {  //  Show values above Alarm  boundary
       
            if(ack_flag != 0) {   // ****  Blink until Ack seen  ***
                uxMessagesWaiting = uxQueueMessagesWaiting(xQblink);
                if(uxQueueMessagesWaiting){
                    xQueuePeek(xQblink, &blink_buffer, 0);
           
                if(blink_buffer == 0) show_seven_seg(blanked);
                else show_seven_seg(measured);
                 
                // ** now check Ack switch to acknowledge blinking **
                uxMessagesWaiting = uxQueueMessagesWaiting(xQsw4);
                if(uxMessagesWaiting)  // check for ACK, turn ACK flag off
                    xQueueReceive(xQsw4, &sw4_buffer, portMAX_DELAY);
                    sw4_state = sw4_buffer;
                    if (sw4_state == 0) {
                        ack_flag = 0;
                        }
                 }  // end if(uxMessageWaiting)
            }   //  end if(ack_flag != 0)
            else show_seven_seg(measured);  // if measure >= ALARM_BOUNDARY
        }   //  End  Show values above Alarm  boundary

        vTaskDelay(ms_delay237);  // check adc value every 600 ms
    }  // End while (true)    
}
 

/**
 * @brief Repeatedly flash the Pico's built-in LED.
 *  Time delay of Flash defined by TaskDelayUntil()  
 */
void blink_task(void* unused_arg) {
    
    int steps = 600;
    uint32_t pico_led_state = 0;
    
   // Initialize start time for vTaskDelayUntil
    TickType_t lastTickTime = xTaskGetTickCount();
        
    while (true) {
 
        pico_led_state = !pico_led_state ;  // Toggle pico led state

        gpio_put(PICO_LED_PIN, pico_led_state); 
        xQueueOverwrite(xQblink, &pico_led_state);
        vTaskDelayUntil(&lastTickTime, pdMS_TO_TICKS(steps));
        }
} 


/**
 * @brief Alternately flash Right and Left Dots on Seven Seg Display
 *        based on the value passed via the inter-task queue.
 *        
 */
void gpio_led_task(void* unused_arg) {  
    // This variable will take a copy of the value
    // added to the FreeRTOS Queue
    uint32_t passed_value_buffer = 0;
    UBaseType_t uxMessagesWaiting  = 0;
    
    while (true) {
        // Check for an item in the FreeRTOS Queue
        uxMessagesWaiting = uxQueueMessagesWaiting(xQblink);
        if(uxQueueMessagesWaiting){
            xQueuePeek(xQblink, &passed_value_buffer,0);
            
            // Received a value so flash DOTL and DOTR accordingly
            gpio_put(D6_PIN, passed_value_buffer == 0 ? 0 : 1);
            gpio_put(D7_PIN, passed_value_buffer == 1 ? 0 : 1);
        }
        vTaskDelay(ms_delay100);  // check Queue input every 100 ms
   }
}



/**
 * @brief Switch Debounce Repeat check of SW, send result to miso led pin task to 
 * stop and start blinking
 * Measures sw state, compares NOW state with PREVIOUS state. If states are different
 * sets count == 0 and looks for two states the same.  It then looks for three or more (MIN_COUNT)
 * in a row or more where NOW and PREVIOUS states are the same. Then Switch state is used
 * as a control signal, passed to an action function by a Queue.
 */
 void sw4_debounce_task(void* unused_arg) {
    uint32_t sw_state = 1;            // initialize sw4_state
    uint32_t sw_previous_state = 1;   // initialize sw4_previous_state
    uint32_t count = 5;               // initialize sw4_final_state
    uint8_t pcfbuffer[]={0b11111111,0b11111111};// data buffer, must be two bytes
    
    xQueueOverwrite(xQsw4, &sw_state);
    while (true) {
        // Measure SW and add the LED state
        // to the FreeRTOS xQUEUE if switch has changed
        sw_previous_state = sw_state;
        i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
        sw_state = readBit(pcfbuffer[0],SW4);
         
        if(sw_previous_state == sw_state) {
            if (count < 12) {
                count += 1;
            }
            else {  // reset cout to MIN_COUNT
                count = MIN_COUNT;
             }              //  End if (count < 12)
         }
        else  { //  if sw_previous state |= sw_state switch has changed
        
             count = 0;  // Need at least MIN_COUNT consecutive same states
             while(count < MIN_COUNT) {
                sw_previous_state = sw_state;
                i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
                sw_state = readBit(pcfbuffer[0],SW4); 
                if(sw_previous_state == sw_state){
                     count++;
                }
                 else {
                     count = 0;
                 }
                vTaskDelay(ms_delay10);  // check switch state every 10 ms
            }
            xQueueOverwrite(xQsw4, &sw_state);
        }   // end else(sw_previous_state |= sw_state)
        vTaskDelay(ms_delay50);  // check switch state every 50 ms
    }  // End while (true)    
}

/**
 * @brief Initialize GPIO Pins for input and output.
 *        Initialize seven segment display
 *        Initialize pcf8575 GPIO Extender
 */
void configure_gpio(void) {
    uint8_t pico_led_state = 0;

    // Configure PICO_LED_PIN for Initialization failure warning
    gpio_init(PICO_LED_PIN);
    gpio_disable_pulls(PICO_LED_PIN);  // remove pullup and pulldowns
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
    
    // Configure D6_PIN for led_task_gpio
    gpio_init(D6_PIN);
    gpio_disable_pulls(D6_PIN);  // remove pullup and pulldowns
    gpio_set_dir(D6_PIN, GPIO_OUT);
    
    // Configure D7_PIN for led_task_gpio 
    gpio_init(D7_PIN);
    gpio_disable_pulls(D7_PIN);  // remove pullup and pulldowns
    gpio_set_dir(D7_PIN, GPIO_OUT);


    // Configure ADC
    adc_init();  
    adc_gpio_init(26);   
    adc_select_input(0);

    // Configure GPIO Extender
    pcf8575_init();

    // Configure Seven Segment display
    config_seven_seg();

}


/*
 * RUNTIME START
 */
int main() {
    uint32_t error_state = 0;
    uint8_t pico_led_state = 0;
    
    stdio_usb_init(); 
    // Pause to allow the USB path to initialize
    sleep_ms(2000);
    
    configure_gpio();
    
        // label Program Screen
    printf("\x1B[2J");  // Clear Screen
    printf("\x1B[%i;%iH", 2,3);  // place curser
    printf("*** Play Program ***");
    printf("\x1B[%i;%iH",4,2);  // place curser
    printf("**************************************\n");
    printf("\x1B[%i;%ir",5,18); // set window top and bottom lines
    printf("\x1B[%i;%iH",5,0);  // place curser

  
    // Set up tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t blink_status = xTaskCreate(blink_task, 
                                         "BLINK_TASK", 
                                         128, 
                                         NULL, 
                                         8, 
                                         &blink_task_handle);
        if (blink_status != pdPASS) {
            error_state  += 1;
            }
             
    BaseType_t adc_status = xTaskCreate(adc_task,              
                                         "ADC_TASK", 
                                         256, 
                                         NULL, 
                                         7,     // Task priority
                                         &adc_task_handle);
        if (adc_status != pdPASS) {
            error_state  += 1;
            }
             
    BaseType_t gpio_status = xTaskCreate(gpio_led_task, 
                                         "GPIO_LED_TASK", 
                                         128, 
                                         NULL, 
                                         6, 
                                         &gpio_led_task_handle);
    
        if (gpio_status != pdPASS) {
            error_state  += 1;
            }
             
    BaseType_t sw4_status = xTaskCreate(sw4_debounce_task, 
                                         "SW4_DEBOUNCE_TASK", 
                                         256, 
                                         NULL, 
                                         5,     // Task priority
                                         &sw4_debounce_task_handle);
        if (sw4_status != pdPASS) {
           error_state  += 1;
            }
            
    
    // Set up the two queues
     
    xQblink = xQueueCreate(1, sizeof(uint32_t));
        if ( xQblink == NULL ) error_state += 1;

    xQsw4 = xQueueCreate(1, sizeof(uint32_t)); 
    if ( xQsw4 == NULL ) error_state += 1;

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (error_state == 0) {
        vTaskStartScheduler();
    }
    else {   // if tasks don't initialize, pico board led will light   
        pico_led_state = 1;
        gpio_put(PICO_LED_PIN, pico_led_state);
    }
    
    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
