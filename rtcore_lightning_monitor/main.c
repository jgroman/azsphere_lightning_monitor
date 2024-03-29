/***************************************************************************//**
* @file    main.c
* @version 1.0.0
*
* Azure Sphere RTCore app driver for lightning detecting TA7642 based AM radio
* receiver.
*
* Parts of code Copyright (c) 2019 Microsoft Corporation. All rights reserved.
* Copyright (c) 2019 Jaroslav Groman
*
* Licensed under the MIT License.
*
*******************************************************************************/

#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-uart-poll.h"
#include "mt3620-timer-poll.h"
#include "mt3620-adc.h"
#include "mt3620-timer.h"
#include "mt3620-timer-user.h"
#include "mt3620-gpio.h"

// Import project hardware abstraction
#include "../hardware/avnet_mt3620_sk/inc/hw/project_hardware.h"

/*******************************************************************************
*   External variables
*******************************************************************************/

extern uint32_t StackTop;   // &StackTop == end of TCM

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define APP_NAME "RTCore Lightning Monitor"

// Select which Click Socket (1 or 2) the TA7642 receiver module is connected to
// TA7642 PWM - Socket PWM (GPIO 0 / 1)
// TA7642 OUT - Socket AN  (ADC0 ch 1 / 2)
// TA7642 GND - Socket GND
#define TA7642_SOCKET_NUMBER 1

#if TA7642_SOCKET_NUMBER == 1
#define TA7642_GPIO_PWM     0
#define TA7642_ADC_CH       1
#else
#define TA7642_GPIO_PWM     1
#define TA7642_ADC_CH       2
#endif // TA7642_SOCKET_NUMBER == 1

// Maximum useable TA7642 signal amplitude before clipping
// signal mV = (signal uint32 * 2500) / 0xFFF
// In case your module output cannot be successfully calibrated, try to slightly
// lower this value, but ideally this should be as close as possible to 1802.
#define TA7642_ADC_MAX  1802     // ~ 1.1 V at 2.5V Vref

// The minimum difference between two output measurements to be considered
// as detected impulse. Setting this affects detector sensitivity - higher
// value means less sensitivity, but better interference immunity.
#define TA7642_NOISE_THRSHLD    70

// Maximum possible detection level value
#define TA7642_LEVEL_THRSHLD_MAX  7000

// Maximum allowed counter value for PWM duty timer
// 40  ~ 500 Hz at 32 kHz timer clock
// 64  ~ 330 Hz at 32 kHz timer clock
// 100 ~ 215 Hz at 32 kHz timer clock
// Try to adjust this value so that calibrated PWM value is approximately 
// at least one half of this.
#define PWM_DUTY_MAX   100

// Calculate optimal operational PWM duty cycle from calibrated PWM duty
// Experiment with this function to find the best detecting experience.
// Try also e.g. (pwm_max / 2) or ((pwm_max / 4) * 3)
#define PWM_TUNE_UP(pwm_max) ((pwm_max / 3) * 2)

// String CR + LF.
#define STRING_CRLF  "\r\n"

// One second in milliseconds
#define MS_1_SECOND 1000

// One hour in seconds
#define S_1_HOUR    3600
 
// Uncomment this directive to enable UART debug messages output
#define DEBUG_ENABLED

// Uncomment this directive to enable intercore messages debug output
//#define DEBUG_MSG_ENABLED

// Index of first payload byte in incoming intercore message
#define PAYLOAD_START_IDX   20

// Max buffer size for intercore messages from and to A7 core
#define MAX_APP_BUFFER_SIZE 256

// Intercore message indentifiers. This is the first byte of incoming message.
#define RTCORE_MSG_DATA_REQUEST     'D'

typedef struct
{
    // PWM duty control variable. Allowed values: 0 - PWM_DUTY_MAX
    // PWM duty ratio is directly controlled by changing this value
    // 1 byte
    uint8_t pwm_duty_ctrl;

    // Current voltage on TA7642 output
    // 4 bytes
    uint32_t ta7642_output;

    // Average voltage on TA7642 output
    // 4 bytes
    uint32_t ta7642_output_avg;

    // Impulse detections per second x 32
    // 2 bytes
    uint16_t detections_1sec_m32;

    // Current warning level
    // 2 bytes
    uint16_t warning_level;

    // Time since the last impulse detection. After one hour with no detection
    // PWM is recalibrated
    // 2 bytes
    uint16_t idle_timer_sec;

} detector_data_t;

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/** 
 * @brief Empty exception handler.
 */
static _Noreturn void
default_exception_handler(void);

/** 
 * @brief Main application entry point.
 */
static _Noreturn void
rtcore_main(void);

/** 
 * @brief Start GPT timer for PWM.
 *
 * Handles starting GPT timer for PWM usage including special cases for 
 * certain counter values.
 */
static void
start_pwm_timer(uint32_t counter);

/**
 * @brief IRQ handler for PWM GPT timer.
 */
static void
handle_irq_pwm_timer(void);

/**
 * @brief Calibrate PWM duty ratio to obtain best TA7642 sensivity.
 *
 * Finds the lowest possible duty ratio (~ operating voltage) when TA7642 starts
 * working with full amplification. This is determined by measuring 1.1 V on 
 * the module output pin.
 * The result is stored in global PWM control g_pwm_duty_ctrl.
 */
static void
calibrate_pwm(void);

/**
 * @brief Process incoming intercore A7 message.
 */
static void
process_a7_message(void);

#ifdef DEBUG_MSG_ENABLED
/**
 * @brief Print bytes from buffer.
 */
static void
print_bytes(const uint8_t *buf, int start, int end);

/**
 * @brief Print formatted application Component Id.
 */
static void
print_guid(const uint8_t *guid);

/**
 * @brief Print formatted intercore A7 message.
 */
static void
debug_a7_message(uint8_t *buffer, uint32_t data_length);
#endif

/*******************************************************************************
* Global variables
*******************************************************************************/

/*
ARM DDI0403E.d SB1.5.2-3
From SB1.5.3, "The Vector table must be naturally aligned to a power of two
whose alignment value is greater than or equal to (Number of Exceptions 
supported x 4), with a minimum alignment of 128 bytes.". The array is aligned 
in linker.ld, using the dedicated section ".vector_table".
*/

// The exception vector table contains a stack pointer, 15 exception handlers, 
// and an entry for each interrupt.
#define INTERRUPT_COUNT     100 // from datasheet
#define EXCEPTION_COUNT     (16 + INTERRUPT_COUNT)
#define INT_TO_EXC(i_)      (16 + (i_))
static const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT]
    __attribute__((section(".vector_table"))) __attribute__((used)) = {
        [0] = (uintptr_t)&StackTop,                  // Main Stack Pointer (MSP)
        [1] = (uintptr_t)rtcore_main,                // Reset
        [2] = (uintptr_t)default_exception_handler,  // NMI
        [3] = (uintptr_t)default_exception_handler,  // HardFault
        [4] = (uintptr_t)default_exception_handler,  // MPU Fault
        [5] = (uintptr_t)default_exception_handler,  // Bus Fault
        [6] = (uintptr_t)default_exception_handler,  // Usage Fault
        [11] = (uintptr_t)default_exception_handler, // SVCall
        [12] = (uintptr_t)default_exception_handler, // Debug monitor
        [14] = (uintptr_t)default_exception_handler, // PendSV
        [15] = (uintptr_t)default_exception_handler, // SysTick

        [INT_TO_EXC(0)] = (uintptr_t)default_exception_handler,
        [INT_TO_EXC(1)] = (uintptr_t)Gpt_HandleIrq1,
        [INT_TO_EXC(2)... INT_TO_EXC(INTERRUPT_COUNT - 1)] = 
            (uintptr_t)default_exception_handler };


// Intercore shared buffers
static BufferHeader *gp_outbound, *gp_inbound;
static uint32_t g_shared_buf_size = 0;

// Application intercore message buffer
static uint8_t g_msg_buffer[MAX_APP_BUFFER_SIZE];
static uint32_t g_msg_byte_count;

// Detector values shared using intercore messages
static detector_data_t g_detector_data;

/*******************************************************************************
* Application entry point
*******************************************************************************/

static _Noreturn void 
rtcore_main(void)
{
    // Shortcut to g_detector_data
    detector_data_t *dd = &g_detector_data;

    // Difference between current and previous voltage on TA7642 output
    // Used to detect impulses over TA7642_NOISE_THRSHLD
    int32_t output_delta;

    uint16_t level_decay = 0;

    // SCB->VTOR = ExceptionVectorTable
    WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

    // Reset GPT2 timer, set 1 kHz clock
    Gpt2_LaunchTimer(0, 0);

#   ifdef DEBUG_ENABLED
    // UART initialization and app header
    Uart_Init();
    Uart_WriteStringPoll(STRING_CRLF);
    Uart_WriteStringPoll(APP_NAME " ("__DATE__ ", " __TIME__")" STRING_CRLF);
#   endif // DEBUG_ENABLED

    // Initialize GPT
    Gpt_Init();

    // Initialize GPIO
    // -- GPIO Block for GPIO0 and GPIO1 - Click Socket 1 & 2 PWM pins
    static const GpioBlock pwm0 = {
        .baseAddr = 0x38010000,
        .type = GpioBlock_PWM,
        .firstPin = 0,
        .pinCount = 4
    };
    Mt3620_Gpio_AddBlock(&pwm0);

    // -- Configure Click Socket PWM pin for TA7642 module PWM input
    Mt3620_Gpio_ConfigurePinForOutput(TA7642_GPIO_PWM);

    // -- Block includes App LED GPIO4.
    static const GpioBlock pwm1 = {
        .baseAddr = 0x38020000,
        .type = GpioBlock_PWM,
        .firstPin = 4,
        .pinCount = 4
    };
    Mt3620_Gpio_AddBlock(&pwm1);

    // -- Configure App LED GPIO4 for output
    Mt3620_Gpio_ConfigurePinForOutput(PROJECT_APP_LED);

    // Initialize ADC
    EnableAdc();

    // Initialize intercore comm buffers
    while (GetIntercoreBuffers(&gp_outbound, &gp_inbound, &g_shared_buf_size) == -1)
    {
        // Empty block, waiting for buffers to become available
    }

    // Calibrate PWM for AGC (Automatic Gain Control)
    calibrate_pwm();
    dd->ta7642_output_avg = dd->ta7642_output;

    // Set default detector values
    dd->detections_1sec_m32 = 0;
    dd->warning_level = 255;
    dd->idle_timer_sec = 0;

    // Store value of free-running GPT2 millisecond timer
    // for measuring integration intervals
    uint32_t last_gpt2_value = Gpt2_GetValue();

    for (;;)
	{
        // Read current TA7642 output voltage
        dd->ta7642_output = ReadAdc(TA7642_ADC_CH);
        dd->ta7642_output_avg = 
            ((dd->ta7642_output_avg * 15) + dd->ta7642_output) / 16;

        output_delta = (int32_t)dd->ta7642_output_avg - 
            (int32_t)dd->ta7642_output;

        if (output_delta >= TA7642_NOISE_THRSHLD)
        {
#           ifdef DEBUG_ENABLED
            Uart_WriteIntegerPoll(output_delta);
            Uart_WriteStringPoll(" ");
#           endif // DEBUG_ENABLED

            // Noise detected, might be lightning
            // Switch App LED on
            Mt3620_Gpio_Write(PROJECT_APP_LED, false);

            // Increase strike counter
            dd->detections_1sec_m32 += 32;
            if (dd->detections_1sec_m32 >= 250)
            {
                // Clip detections at 8 max, at this count it is probably
                // just interference instead of lightning
                dd->detections_1sec_m32 = 250;
            }
        }
        else
        {
        // Switch App LED off
        Mt3620_Gpio_Write(PROJECT_APP_LED, true);
        }

        if (Gpt2_GetValue() - last_gpt2_value >= MS_1_SECOND)
        {
            // One second has passed since the last results update

#           ifdef DEBUG_ENABLED
            // output_delta debug output formatting
            if (dd->detections_1sec_m32 > 0)
            {
                Uart_WriteStringPoll(STRING_CRLF);
            }
#           endif // DEBUG_ENABLED

            // If there were at least two impulses, increase warning level
            if (dd->detections_1sec_m32 > 32)
            {
                dd->warning_level += dd->detections_1sec_m32;
            }

            // Warning level decay during time
            level_decay = (dd->warning_level >> 8);
            dd->warning_level = dd->warning_level - level_decay;

            if (dd->warning_level > TA7642_LEVEL_THRSHLD_MAX)
            {
                // Clip warning level to allowed maximum
                dd->warning_level = TA7642_LEVEL_THRSHLD_MAX;
            }

            if (level_decay == 0)
            {
                dd->idle_timer_sec++;

                if (dd->idle_timer_sec >= S_1_HOUR)
                {
                    // No detections for one hour, recalibrate PWM.
                    dd->idle_timer_sec = 0;
#                   ifdef DEBUG_ENABLED
                    Uart_WriteStringPoll("Recalibrating PWM after 1 hour idle." 
                        STRING_CRLF);
#                   endif // DEBUG_ENABLED
                    calibrate_pwm();
                }
            }
            else
            {
                // Reset idle timer
                dd->idle_timer_sec = 0;
            }

#           ifdef DEBUG_ENABLED
            Uart_WriteStringPoll("PWM:");
            Uart_WriteIntegerPoll(dd->pwm_duty_ctrl);
            Uart_WriteStringPoll(" OUT:");
            Uart_WriteIntegerPoll(dd->ta7642_output);
            Uart_WriteStringPoll(" AVG:");
            Uart_WriteIntegerPoll(dd->ta7642_output_avg);
            Uart_WriteStringPoll(" DLT:");
            Uart_WriteIntegerPoll(output_delta);
            Uart_WriteStringPoll(" CNT:");
            Uart_WriteIntegerPoll(dd->detections_1sec_m32);
            Uart_WriteStringPoll(" LVL:");
            Uart_WriteIntegerPoll(dd->warning_level);
            Uart_WriteStringPoll(" DCY:");
            Uart_WriteIntegerPoll(level_decay);
            Uart_WriteStringPoll(STRING_CRLF);
#           endif // DEBUG_ENABLED

            // Reset impulse counter
            dd->detections_1sec_m32 = 0;

            // Store current millisecond timer value
            last_gpt2_value = Gpt2_GetValue();
        }

        // Read incoming data from A7 Core
        process_a7_message();
    }
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static _Noreturn void
default_exception_handler(void)
{
    for (;;)
    {
        // Empty Block
    }
}

static void
calibrate_pwm(void)
{
    // Calibrate PWM control value
    // We are looking for the lowest possible duty ratio (~ operating voltage)
    // when TA7642 starts working with full amplification. This is determined
    // by measuring 1.1 V on the module output pin

    detector_data_t *dd = &g_detector_data;

    bool b_state_app_led = false;       // APP LED state flag
    bool b_is_locked_app_led = false;   // APP LED state mutable flag

#   ifdef DEBUG_ENABLED
    Uart_WriteStringPoll("Calibrating PWM ..." STRING_CRLF);
#   endif // DEBUG_ENABLED

    // Warning level value 0 is used as a flag that calibration is in progress
    dd->warning_level = 0;
    
    dd->pwm_duty_ctrl = 0;
    start_pwm_timer(dd->pwm_duty_ctrl);   // PWM pin Switch off
    Gpt2_WaitMs(250);                     // Wait for full capacitor discharge

    // Switch on App LED
    Mt3620_Gpio_Write(PROJECT_APP_LED, b_state_app_led);

    bool b_is_not_calibrated = true;
    do
    {
        // Increase PWM duty until there is full signal on ADC input
        // or PWM duty is on max level
        for (uint8_t pwm_duty = 1; pwm_duty <= PWM_DUTY_MAX; pwm_duty++)
        {
            dd->pwm_duty_ctrl = pwm_duty;  // Update PWM duty ratio
            Gpt2_WaitMs(250);              // Wait for capacitor charging

            // Flip App LED state to indicate progress
            if (!b_is_locked_app_led)
            {
                b_state_app_led = !b_state_app_led;
                Mt3620_Gpio_Write(PROJECT_APP_LED, b_state_app_led);
            }

            // Measure TA7642 module output voltage
            dd->ta7642_output = ReadAdc(TA7642_ADC_CH);
            if (dd->ta7642_output > TA7642_ADC_MAX)
            {
                // Voltage on TA7642 module output has reached required value
                // Calibration was successful
                b_is_not_calibrated = false;
                break;
            }

            // Read incoming data from A7 Core
            process_a7_message();
        }

        // There was at least one full PWM cycle without successful calibration
        // Leave App LED on to indicate problem
        if (b_is_not_calibrated)
        {
            b_is_locked_app_led = true;
            b_state_app_led = false;        // LED output is active Low
            Mt3620_Gpio_Write(PROJECT_APP_LED, b_state_app_led);

#           ifdef DEBUG_ENABLED
            Uart_WriteStringPoll("ERROR: *** Calibration failed." STRING_CRLF);
#           endif // DEBUG_ENABLED
        }

    } while (b_is_not_calibrated);

    // Switch App LED off
    b_state_app_led = true;        // LED output is active Low
    Mt3620_Gpio_Write(PROJECT_APP_LED, b_state_app_led);

#   ifdef DEBUG_ENABLED
    Uart_WriteStringPoll("Calibrated at PWM ");
    Uart_WriteIntegerPoll(dd->pwm_duty_ctrl);
    Uart_WriteStringPoll(STRING_CRLF);
#   endif // DEBUG_ENABLED

    // "Tune up" operational PWM duty to be certain percentage of calibrated 
    // value. See PWM_TUNE_UP macro.
    dd->pwm_duty_ctrl = PWM_TUNE_UP(dd->pwm_duty_ctrl);

    Gpt2_WaitMs(500);   // Wait for capacitors

    // Restore default warning level value
    dd->warning_level = 255;

    return;
}

static void
start_pwm_timer(uint32_t counter)
{
    // GPT timer doesn't work for counter values 0 and 1 so we have to
    // skip using it for those values

    if (counter == 0)
    {
        handle_irq_pwm_timer();     // Skip timer, start IRQ handler
    }
    else if (counter == 1)
    {
        // Wait approx one pulse length
        Gpt3_WaitUs(70);

        handle_irq_pwm_timer();     // Start IRQ handler
    }
    else   // Counter is  > 1
    {
        if (counter > PWM_DUTY_MAX)
        {
            counter = PWM_DUTY_MAX;
        }
        // Start timer normally
        Gpt_LaunchTimer32k(TimerGpt0, counter, handle_irq_pwm_timer);
    }
}


static void
handle_irq_pwm_timer(void)
{
    static bool b_is_pwm_gpio_high = true;

    // Calculate this pulse duration
    uint32_t counter = (b_is_pwm_gpio_high) ?
        g_detector_data.pwm_duty_ctrl : 
        PWM_DUTY_MAX - g_detector_data.pwm_duty_ctrl;

    // Do not change output GPIO state if requested pulse duration is 0
    if (counter > 0)
    {
        Mt3620_Gpio_Write(TA7642_GPIO_PWM, b_is_pwm_gpio_high);
    }

    // Prepare the next PWM pulse state
    b_is_pwm_gpio_high = !b_is_pwm_gpio_high;

    // Start this pulse timer
    start_pwm_timer(counter);
}

static void
process_a7_message(void)
{
    // Read incoming data from A7 Core
    // On success, g_msg_byte_count is set to the number of read bytes.
    int dequeue_result = DequeueData(gp_outbound, gp_inbound,
        g_shared_buf_size, g_msg_buffer, &g_msg_byte_count);

    if (dequeue_result != -1 && g_msg_byte_count >= PAYLOAD_START_IDX)
    {
        // Received complete message from A7 Core

#       ifdef DEBUG_MSG_ENABLED
        Uart_WriteStringPoll("Incoming ");
        debug_a7_message(g_msg_buffer, g_msg_byte_count);
#       endif // DEBUG_MSG_ENABLED

        if (g_msg_buffer[PAYLOAD_START_IDX] == RTCORE_MSG_DATA_REQUEST)
        {
            // Received data request

            // Leave the first byte of message as-is. This is used for detecting
            // message type in A7 core

            // Copy g_detector_data to message buffer
            memcpy(g_msg_buffer + PAYLOAD_START_IDX + 1, &g_detector_data, 
                sizeof(detector_data_t));

#           ifdef DEBUG_MSG_ENABLED
            Uart_WriteStringPoll("Outgoing ");
            debug_a7_message(g_msg_buffer, 
                PAYLOAD_START_IDX + sizeof(detector_data_t) + 1);
#           endif // DEBUG_MSG_ENABLED

            // Send message buffer to A7 Core
            EnqueueData(gp_inbound, gp_outbound, g_shared_buf_size, g_msg_buffer,
                PAYLOAD_START_IDX + 1 + sizeof(g_detector_data));
        }
    }

    return;
}

#ifdef DEBUG_MSG_ENABLED
static void
print_bytes(const uint8_t *buf, int start, int end)
{
    int step = (end >= start) ? +1 : -1;

    for (/* nop */; start != end; start += step)
    {
        Uart_WriteHexBytePoll(buf[start]);
    }
    Uart_WriteHexBytePoll(buf[end]);
}

static void
print_guid(const uint8_t *guid)
{
    print_bytes(guid, 3, 0); // 4-byte little-endian word
    Uart_WriteStringPoll("-");
    print_bytes(guid, 5, 4); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    print_bytes(guid, 7, 6); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    print_bytes(guid, 8, 9); // 2 bytes
    Uart_WriteStringPoll("-");
    print_bytes(guid, 10, 15); // 6 bytes
}

static void
debug_a7_message(uint8_t *buffer, uint32_t data_length)
{
    // Print incoming message header
    Uart_WriteStringPoll("A7 message of ");
    Uart_WriteIntegerPoll(data_length);
    Uart_WriteStringPoll("bytes:" STRING_CRLF);

    // Print the Component Id (A7 Core)
    Uart_WriteStringPoll("  Component Id (16 bytes): ");
    print_guid(buffer);
    Uart_WriteStringPoll(STRING_CRLF);

    // Print reserved field as little-endian 4-byte integer.
    Uart_WriteStringPoll("  Reserved (4 bytes): ");
    print_bytes(buffer, 19, 16);
    Uart_WriteStringPoll(STRING_CRLF);

    // Print message as hex.
    size_t payloadBytes = data_length - PAYLOAD_START_IDX;
    Uart_WriteStringPoll("  Payload (");
    Uart_WriteIntegerPoll(payloadBytes);
    Uart_WriteStringPoll(" bytes as hex): ");

    for (size_t i = PAYLOAD_START_IDX; i < data_length; ++i)
    {
        Uart_WriteHexBytePoll(buffer[i]);
        if (i != data_length - 1)
        {
            Uart_WriteStringPoll(":");
        }
    }
    Uart_WriteStringPoll(STRING_CRLF);

    // Print message as text.
    Uart_WriteStringPoll("  Payload (");
    Uart_WriteIntegerPoll(payloadBytes);
    Uart_WriteStringPoll(" bytes as text): ");
    char printed_char_str[2];
    printed_char_str[1] = '\0';
    for (size_t i = PAYLOAD_START_IDX; i < data_length; ++i)
    {
        printed_char_str[0] = isprint(buffer[i]) ? buffer[i] : '.';
        Uart_WriteStringPoll(printed_char_str);
    }
    Uart_WriteStringPoll(STRING_CRLF);
}
#endif

/* [] END OF FILE */
