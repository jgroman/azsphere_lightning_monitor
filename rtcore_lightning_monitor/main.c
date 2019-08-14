/***************************************************************************//**
* @file    main.c
* @version 1.0.0
*
* Original work Copyright (c) 2019 Microsoft Corporation. All rights reserved.
* Modified work Copyright (c) 2019 Jaroslav Groman
*
* Licensed under the MIT License.
*
*******************************************************************************/

#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-uart-poll.h"
#include "mt3620-adc.h"
#include "mt3620-timer-2.h"
#include "mt3620-gpio.h"

// Import project hardware abstraction
#include "../hardware/avnet_mt3620_sk/inc/hw/project_hardware.h"

/*******************************************************************************
*   External variables
*******************************************************************************/

extern uint32_t StackTop;   // &StackTop == end of TCM0

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
#define TA7642_ADC_MAX  3276     // ~ 2.0 V at 2.5V Vref

// Maximum allowed counter value for PWM duty timer
#define PWM_DUTY_MAX   40        // ~ 500 Hz at 32 kHz timer clock


// String CR + LF.
#define STRING_CRLF  "\r\n"
 
// Uncomment this directive to enable UART debug messages output
#define DEBUG_ENABLED

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

static void
handle_irq_gpt(void);

/** 
 * @brief Print bytes from buffer.
 */
static void
print_bytes(const uint8_t *buf, int start, int end);

static void 
print_guid(const uint8_t *guid);

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


// PWM duty control variable. Allowed values: 0 - PWM_DUTY_MAX
// PWM duty ratio is directly controlled by changing this value
static uint8_t g_pwm_duty_ctrl;

/*******************************************************************************
* Application entry point
*******************************************************************************/

static _Noreturn void 
rtcore_main(void)
{
	union Analog_data
	{
		uint32_t u32;
		uint8_t u8[4];
	} ADC_data;

    // SCB->VTOR = ExceptionVectorTable
    WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

    // Reset GPT2 timer, set 1 kHz clock
    Gpt2_LaunchTimer(0, 0);

#   ifdef DEBUG_ENABLED
    // UART initialization and app header
    Uart_Init();
    Uart_WriteStringPoll(APP_NAME " ("__DATE__ ", " __TIME__")" STRING_CRLF);
    Uart_WriteIntegerPoll(Gpt2_GetValue());
    Uart_WriteStringPoll(STRING_CRLF);

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

    // -- Block includes led1RedGpio, GPIO8.
    static const GpioBlock pwm2 = {
        .baseAddr = 0x38030000,
        .type = GpioBlock_PWM,
        .firstPin = 8,
        .pinCount = 4 
    };
    Mt3620_Gpio_AddBlock(&pwm2);

    Mt3620_Gpio_ConfigurePinForOutput(PROJECT_RGBLED_RED);
    Mt3620_Gpio_Write(PROJECT_RGBLED_RED, false);

    // Initialize ADC
    EnableAdc();

    // Initialize intercore comm buffers
    BufferHeader *outbound, *inbound;
    uint32_t sharedBufSize = 0;
    while (GetIntercoreBuffers(&outbound, &inbound, &sharedBufSize) == -1)
    {
        // Empty block, waiting for buffers to become available
    }

    // Start PWM timer

    // Calibrate PWM control value
    Uart_WriteStringPoll("Setup PWM" STRING_CRLF);
    g_pwm_duty_ctrl = 30;
    start_pwm_timer(g_pwm_duty_ctrl);     // PWM pin Switch off
    Gpt2_WaitMs(250);                     // Wait for full capacitor discharge


    for (;;) {
        __asm__("wfi");
    }


    do
    {

    } while (true);


    bool is_calibrated = false;
    for (uint8_t pwm_duty = 1; pwm_duty <= PWM_DUTY_MAX; pwm_duty++)
    {
        // Increase PWM duty until there is full signal on ADC input
        // or PWM duty is on max level

        g_pwm_duty_ctrl = pwm_duty;      // Update PWM duty ratio
        Gpt2_WaitMs(250);                // Wait for capacitor charging

        ADC_data.u32 = ReadAdc(TA7642_ADC_CH);

    }

    if (!is_calibrated)
    {

    }

    Uart_WriteIntegerPoll(Gpt2_GetValue());
    Uart_WriteStringPoll(STRING_CRLF);


    Uart_WriteStringPoll("GPT2: ");
    Uart_WriteIntegerPoll(Gpt2_GetValue());
    Uart_WriteStringPoll(STRING_CRLF);

    /*
    for (;;) {
        __asm__("wfi");
    }
    */

    // Main program loop
#   define PAYLOAD_START_IDX   20
#   define MAX_APP_BUFFER_SIZE 256
    uint8_t app_buf[MAX_APP_BUFFER_SIZE];
    uint32_t read_bytes_count = sizeof(app_buf);
    for (;;)
	{
        // Read incoming data from A7 Core
        // On success, read_bytes_count is set to the number of read bytes.
        int dequeue_result = DequeueData(outbound, inbound, sharedBufSize, 
            app_buf, &read_bytes_count);

        if (dequeue_result == -1 || read_bytes_count < PAYLOAD_START_IDX)
		{
            continue;
        }

#       ifdef DEBUG_ENABLED
        // Print incoming message header
        Uart_WriteStringPoll("Received message of ");
        Uart_WriteIntegerPoll(read_bytes_count);
        Uart_WriteStringPoll("bytes:" STRING_CRLF);

        // Print the Component Id (A7 Core)
        Uart_WriteStringPoll("  Component Id (16 bytes): ");
        print_guid(app_buf);
        Uart_WriteStringPoll(STRING_CRLF);

        // Print reserved field as little-endian 4-byte integer.
        Uart_WriteStringPoll("  Reserved (4 bytes): ");
        print_bytes(app_buf, 19, 16);
        Uart_WriteStringPoll(STRING_CRLF);

        // Print message as hex.
        size_t payloadBytes = read_bytes_count - PAYLOAD_START_IDX;
        Uart_WriteStringPoll("  Payload (");
        Uart_WriteIntegerPoll(payloadBytes);
        Uart_WriteStringPoll(" bytes as hex): ");

        for (size_t i = PAYLOAD_START_IDX; i < read_bytes_count; ++i)
        {
            Uart_WriteHexBytePoll(app_buf[i]);
            if (i != read_bytes_count - 1)
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
        for (size_t i = PAYLOAD_START_IDX; i < read_bytes_count; ++i)
        {
            printed_char_str[0] = isprint(app_buf[i]) ? app_buf[i] : '.';
            Uart_WriteStringPoll(printed_char_str);
        }
        Uart_WriteStringPoll(STRING_CRLF);
#       endif // DEBUG_ENABLED

        // Read ADC channel
        uint8_t adc_channel = 1;
        ADC_data.u32 = ReadAdc(adc_channel);

#ifdef DEBUG_ENABLED
        uint32_t mV;
        mV = (ADC_data.u32 * 2500) / 0xFFF;
        Uart_WriteStringPoll("ADC channel ");
        Uart_WriteIntegerPoll(adc_channel);
        Uart_WriteStringPoll(" : ");
        Uart_WriteIntegerPoll(mV / 1000);
        Uart_WriteStringPoll(".");
        Uart_WriteIntegerWidthPoll(mV % 1000, 3);
        Uart_WriteStringPoll(" V");
        Uart_WriteStringPoll(STRING_CRLF);
#endif // DEBUG_ENABLED

        uint8_t analog_buf_idx = 0;
		for (int buf_idx = 0; buf_idx < 4; buf_idx++)
		{
			// Copy ADC data to app buffer
            app_buf[PAYLOAD_START_IDX + buf_idx] = 
                ADC_data.u8[analog_buf_idx++];
		}

		// Send buffer to A7 Core
        EnqueueData(inbound, outbound, sharedBufSize, app_buf, 
            PAYLOAD_START_IDX + 4);
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

    // Calculate this pulse duration counter
    uint32_t counter = (b_is_pwm_gpio_high) ?
        g_pwm_duty_ctrl : PWM_DUTY_MAX - g_pwm_duty_ctrl;

    if (counter > 0)
    {
        // Do not change output GPIO state if requested pulse duration is 0
        Mt3620_Gpio_Write(TA7642_GPIO_PWM, b_is_pwm_gpio_high);
    }

    b_is_pwm_gpio_high = !b_is_pwm_gpio_high;

    start_pwm_timer(counter);
}

static void
handle_irq_gpt(void)
{
 
}

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

/* [] END OF FILE */
