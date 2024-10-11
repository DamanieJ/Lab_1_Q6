//816030907
//Damanie Jangbahadoorsingh

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/uart.h"

#define BUF_SIZE        1024         // Buffer size for UART communication
#define TEST_ONE_SHOT   false        // Timer test configuration for one-shot mode
#define TEST_RELOAD     true         // Timer test configuration for reload mode
#define FOREVER         1            // Constant to represent infinite loops
#define USER_CHAR       'o'          // User character for state transitions
#define ON_STATE        0            // ON state identifier
#define OFF_STATE       1            // OFF state identifier
#define INTERVAL_MAX    5            // Max interval before resetting
#define UART_BAUD_RATE  74880        // UART baud rate
#define TIMER_PERIOD_US 100          // Timer period in microseconds

// Structure to store time values (minutes, seconds, milliseconds)
struct time_store {
    int min;
    int second;
    int millisec;
};



struct time_store current_time;      // Variable to hold current time
int count;                           // Counter for the timer callback
unsigned int events, state, state_events;
struct time_store cooktime, targettick;
uint32_t tps;
uint8_t *key;                        // Pointer to store UART received data
int interval;                        // Interval counter

// Configure GPIO input (GPIO_NUM_0 as input)
void config_input(void) {
    gpio_config_t config_input;

    config_input.mode = GPIO_MODE_DEF_OUTPUT;      // Set GPIO as input
    config_input.pull_up_en = GPIO_PULLUP_DISABLE;
    config_input.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config_input.intr_type = GPIO_INTR_DISABLE;   // Disable GPIO interrupts
    config_input.pin_bit_mask = (1ULL << GPIO_NUM_0);  // Set GPIO_NUM_0 as input
    gpio_config(&config_input);                   // Apply configuration
}

// Initialize time store struct
void init_time_store(struct time_store *time) {
    time->min = 0;
    time->second = 0;
    time->millisec = 0;
}

// Timer callback to increment the current time (called every 100 us)
void hw_timer_callback1(void *arg) {
    count++;

    // Increment milliseconds every 10 counts
    if (count == 10) {
        current_time.millisec++;
        
        if (current_time.millisec == 501) {
            current_time.millisec = 0;    // Reset milliseconds after 500
        }
        count = 0;
    }
}

// Return current time in milliseconds as integer for easy calculations
int time(struct time_store times) {
    return times.millisec;
}

// Check if the clock has ticked by a multiple of 100 ms
bool clock_tick(void) {
    int temp = time(current_time);
    interval++;
    //printf("tick \n");
    if (interval > INTERVAL_MAX) {
        interval = 0;
    }

    return (temp % 100 == 0);  // Return true every 100 ms
}

// Configure the hardware timer
void hw_timer_config(void) {
    hw_timer_init(hw_timer_callback1, NULL);       // Initialize timer with callback
    hw_timer_alarm_us(TIMER_PERIOD_US, TEST_RELOAD); // Set timer period to 100 us with reload
}

// Configure UART with predefined settings
void config_uart(void) {
    uart_config_t config_uart;

    config_uart.baud_rate = UART_BAUD_RATE;         // Set baud rate
    config_uart.data_bits = UART_DATA_8_BITS;       // 8 data bits
    config_uart.parity = UART_PARITY_DISABLE;       // No parity
    config_uart.stop_bits = UART_STOP_BITS_1;       // 1 stop bit
    config_uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // No flow control

    uart_param_config(UART_NUM_0, &config_uart);    // Apply UART configuration
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0); // Install UART driver
}

// Function to receive UART data
uint8_t *serial_receive(void) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);  // Allocate buffer for UART data
    int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 100 / portTICK_RATE_MS);
    uart_write_bytes(UART_NUM_0, (const char *) data, len);
  // Read UART bytes

    uint8_t data2 = data;
    free(data);
    return data2;  // Return pointer to received data
    
}

// Function to handle ON_STATE
void On_State(void) {
    const char *message = "ON_STATE \n";
    uart_write_bytes(UART_NUM_0, message, strlen(message));  // Send ON state message via UART

    state = ON_STATE;  // Set state to ON
    gpio_set_level(GPIO_NUM_2, 1);  // Set GPIO pin to HIGH (ON)

    key = serial_receive(); 
    //printf(const char* key);
 // Receive UART data
}

// Function to handle OFF_STATE
void Off_state(void) {
    const char *message = "OFF_STATE \n";
    uart_write_bytes(UART_NUM_0, message, strlen(message));  // Send OFF state message via UART

    state = OFF_STATE;  // Set state to OFF
    gpio_set_level(GPIO_NUM_2, 0);  // Set GPIO pin to LOW (OFF)

    key = serial_receive();
    printf("%d",interval);
    

      // Receive UART data
}

// Main application function
void app_main(void) {
    uint8_t data2 = (uint8_t)USER_CHAR;  // Store user character as uint8_t

    // Initialize variables and structures
    init_time_store(&current_time);
    count = 0;
    state = ON_STATE;  // Start with ON state

    // Configure peripherals (input, UART, timer)
    config_input();
    config_uart();
    hw_timer_config();

    // Main loop
    while (FOREVER) {
        key = serial_receive();  // Continuously receive UART data
        printf("State = %2d \r", state);  // Print current state

        // Handle state transitions and actions
        switch (state) {
            case ON_STATE:
                if ((key == &data2) && (interval >= INTERVAL_MAX) && clock_tick()) {
                    state = OFF_STATE;  // Transition to OFF state
                } else if ((key != &data2) && clock_tick()) {
                    state = OFF_STATE;  // Handle key mismatch
                } else {
                    On_State();  
                    // Remain in ON state
                }
                break;

            case OFF_STATE:
                if ((key == &data2) && (interval == INTERVAL_MAX) && clock_tick()) {
                    state = ON_STATE;  // Transition to ON state
                } else if ((key != &data2) && clock_tick()) {
                    state = ON_STATE;  // Handle key mismatch
                } else {
                    Off_state();  // Remain in OFF state
                }
                break;

            default:
                // Handle unexpected state
                break;
        }
    }
}
