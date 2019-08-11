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
#include "mt3620-timer.h"
#include "mt3620-gpio.h"

// Import project hardware abstraction
#include "../hardware/avnet_mt3620_sk/inc/hw/project_hardware.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define APP_NAME "RTCore Lightning Monitor"

// Select which Click Socket (1 or 2) the TA7642 radio is connected to
// TA7642 PWM - Socket PWM (GPIO 0 / 1)
// TA7642 OUT - Socket AN  (ADC0 ch 1 / 2)
// TA7642 GND - Socket GND
#define TA7642_SOCKET_NUMBER 1

#define PWM_INTERVAL_MS 50

// String CR + LF.
#define STRING_CRLF  "\r\n"
 
// Uncomment this directive to enable UART debug messages output
#define DEBUG_ENABLED

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/** @brief Empty exception handler.
 *
 */
static _Noreturn void
default_exception_handler(void);

/** @brief Main application.
 *
 */
static _Noreturn void
rtcore_main(void);

/** @brief Print bytes from buffer.
 *
 */
static void
print_bytes(const uint8_t *buf, int start, int end);

static void 
print_guid(const uint8_t *guid);

static void
HandleBlinkTimerIrq(void);

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
extern uint32_t StackTop;   // &StackTop == end of TCM0
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


static bool led1RedOn = false;

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
	} analog_data;

    // SCB->VTOR = ExceptionVectorTable
    WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

#   ifdef DEBUG_ENABLED
    // UART initialization and app header
    Uart_Init();
    Uart_WriteStringPoll(APP_NAME " ("__DATE__ ", " __TIME__")" STRING_CRLF);
#   endif // DEBUG_ENABLED

    // Initialize timers
    Gpt_Init();

    // Block includes GPIO0 and GPIO1 - "official" PWM pins
    static const GpioBlock pwm0 = {
        .baseAddr = 0x38010000,
        .type = GpioBlock_PWM,
        .firstPin = 0,
        .pinCount = 4 
    };
    Mt3620_Gpio_AddBlock(&pwm0);

    // Block includes led1RedGpio, GPIO8.
    static const GpioBlock pwm2 = {
        .baseAddr = 0x38030000,
        .type = GpioBlock_PWM,
        .firstPin = 8,
        .pinCount = 4 
    };
    Mt3620_Gpio_AddBlock(&pwm2);

    Mt3620_Gpio_ConfigurePinForOutput(PROJECT_RGBLED_RED);

    Mt3620_Gpio_Write(PROJECT_RGBLED_RED, led1RedOn);


    // TA7642 PWM GPIO
    Mt3620_Gpio_ConfigurePinForOutput(TA7642_SOCKET_NUMBER - 1);

    Gpt_LaunchTimerMs(TimerGpt0, 500, HandleBlinkTimerIrq);

    for (;;) {
        __asm__("wfi");
    }

	// ADC Initialization
	EnableAdc();

    BufferHeader *outbound, *inbound;
    uint32_t sharedBufSize = 0;
    while (GetIntercoreBuffers(&outbound, &inbound, &sharedBufSize) == -1)
    {
        // Empty block, waiting for buffers to become available
    }

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
        analog_data.u32 = ReadAdc(adc_channel);

#ifdef DEBUG_ENABLED
        uint32_t mV;
        mV = (analog_data.u32 * 2500) / 0xFFF;
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
                analog_data.u8[analog_buf_idx++];
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
HandleBlinkTimerIrq(void)
{
    led1RedOn = !led1RedOn;
    Mt3620_Gpio_Write(PROJECT_RGBLED_RED, led1RedOn);

    Gpt_LaunchTimerMs(TimerGpt0, 500, HandleBlinkTimerIrq);
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
