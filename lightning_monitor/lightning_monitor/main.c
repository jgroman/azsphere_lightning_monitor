/**
 * Copyright (c) 2019 Jaroslav Groman
 *
 * Licensed under the MIT License.
 */

#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

#include <applibs/log.h>
#include <applibs/i2c.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// RTCore app connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>

#include "init.h"
#include "main.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define OLED_LINE_LENGTH    (16u)

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/


/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
volatile sig_atomic_t gb_is_termination_requested = false;

// File descriptors
int g_fd_socket = -1;
int g_fd_epoll = -1;
int g_fd_i2c = -1;

u8x8_t g_u8x8;                                  // OLED device descriptor

/*******************************************************************************
* Main program entry
*******************************************************************************/
int 
main(void)
{
    char oled_buffer[OLED_LINE_LENGTH + 1];

    Log_Debug("\n*** App starting ***\n");

    // Initialize handlers
    if (init_handlers() != 0)
    {
        // Failed to initialize handlers
        gb_is_termination_requested = true;
    }

    // Initialize peripherals
    if (!gb_is_termination_requested && (init_peripherals() != 0))
    {
        // Failed to initialize peripherals
        gb_is_termination_requested = true;
    }

    // Main program loop
    if (!gb_is_termination_requested)
    {

        // Initialize OLED display
        u8x8_InitDisplay(&g_u8x8);
        u8x8_SetPowerSave(&g_u8x8, 0);
        u8x8_SetFont(&g_u8x8, u8x8_font_amstrad_cpc_extended_f);
        u8x8_ClearDisplay(&g_u8x8);

        snprintf(oled_buffer, 16, ".. STARTING ..");
        u8x8_DrawString(&g_u8x8, 0, 0, oled_buffer);


        Log_Debug("Waiting for events.\n");

        // Main program loop
        while (!gb_is_termination_requested)
        {
            // Handle timers
            if (WaitForEventAndCallHandler(g_fd_epoll) != 0)
            {
                // Event polling failed
                gb_is_termination_requested = true;
            }
        }
        Log_Debug("Exiting main loop.\n");
    }

    // Clean up and shutdown
    Log_Debug("Shutting down.\n\n");
    u8x8_ClearDisplay(&g_u8x8);
    init_shutdown();
}
/*******************************************************************************
* Public function definitions
*******************************************************************************/

void
handle_termination(int signal_number)
{
    gb_is_termination_requested = true;
}

void
handle_button1_press(void)
{
    Log_Debug("Button1 pressed.\n");
    gb_is_termination_requested = true;
}

void
handle_button2_press(void)
{
    Log_Debug("Button2 pressed.\n");
    gb_is_termination_requested = true;
}

void
handle_sample_request(void)
{
    static int iter = 0;

    // Send "Read-ADC-%d" message to real-time capable application.
    static char buf_tx[32];

    sprintf(buf_tx, "Read-ADC-%d", iter++);
    Log_Debug("Sending: %s\n", buf_tx);

    int bytesSent = send(g_fd_socket, buf_tx, strlen(buf_tx), 0);
    if (bytesSent == -1)
    {
        Log_Debug("ERROR: Unable to send message: %d (%s)\n", 
            errno, strerror(errno));
        gb_is_termination_requested = true;
        return;
    }
}

void
handle_rtcore_receive(void)
{
    // Read response from real-time capable application.
    char buf_rx[32];

    union Analog_data
    {
        uint32_t u32;
        uint8_t u8[4];
    } analog_data;

    int bytesReceived = recv(g_fd_socket, buf_rx, sizeof(buf_rx), 0);

    if (bytesReceived == -1)
    {
        Log_Debug("ERROR: Unable to receive message: %d (%s)\n",
            errno, strerror(errno));
        gb_is_termination_requested = true;
    }

    Log_Debug("Received %d bytes.\n", bytesReceived);

    // Copy data from Rx buffer to analog_data union
    for (int i = 0; i < sizeof(analog_data); i++)
    {
        analog_data.u8[i] = buf_rx[i];
    }

    // Voltage in Volts = 2.5 * adc_reading / 4096
    double voltage = (2.5 * analog_data.u32) / 4096;

    Log_Debug("Measured voltage: %f V\n", voltage);

    return;
}


/*******************************************************************************
* Private function definitions
*******************************************************************************/







/* [] END OF FILE */
