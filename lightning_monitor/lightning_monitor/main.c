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
#include "support.h"
#include "main.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

static int
rtcore_send(const uint8_t *p_buffer, size_t byte_count);

static int
rtcore_receive(uint8_t *p_buffer, size_t byte_count);

static bool
rtcore_ping(void);

static void
display_screen(uint8_t id_screen);


/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
volatile sig_atomic_t gb_is_termination_requested = false;

int g_fd_socket = -1;       // Socket file descriptor
int g_fd_epoll = -1;        // Epoll file descriptor
int g_fd_i2c = -1;          // I2C interface file descriptor

u8g2_t g_u8g2;

uint8_t g_screen_id = 1;

/*******************************************************************************
* Public function definitions
*******************************************************************************/

/*******************************************************************************
* Main program entry
*******************************************************************************/
int 
main(int argc, char *argv[])
{
    DEBUG("*** App starting ***\n", __FUNCTION__);

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

    // Check that RTCore companion app is running
    if (!gb_is_termination_requested)
    {
        if (!rtcore_ping())
        {
            // RTCore app didn't respond to PING request
            gb_is_termination_requested = true;
            ERROR("RTCore App is not ready.\n", __FUNCTION__);
        }
    }

    // Main program
    if (!gb_is_termination_requested)
    {
        u8g2_ClearDisplay(&g_u8g2);

        DEBUG("Waiting for events.\n", __FUNCTION__);

        // Main program loop
        while (!gb_is_termination_requested)
        {
            display_screen(g_screen_id);

            // Handle timers
            if (WaitForEventAndCallHandler(g_fd_epoll) != 0)
            {
                // Event polling failed
                gb_is_termination_requested = true;
            }
        }
        DEBUG("Exiting main loop.\n", __FUNCTION__);

        u8g2_ClearDisplay(&g_u8g2);
    }

    // Clean up and shutdown
    DEBUG("Shutting down.\n\n", __FUNCTION__);
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
    DEBUG("Button1 pressed.\n", __FUNCTION__);

    if (g_screen_id == 1)
    {
        g_screen_id = 2;
    }
    else
    {
        g_screen_id = 1;
    }

    return;
}

void
handle_button2_press(void)
{
    DEBUG("Button2 pressed.\n", __FUNCTION__);
    gb_is_termination_requested = true;
}

void
handle_request_data(void)
{
    // Request data from RTCore app
    DEBUG("Requesting data from RTCore\n", __FUNCTION__);
    (void)rtcore_send(RTCORE_MSG_REQUEST_DATA, strlen(RTCORE_MSG_REQUEST_DATA));

    // Request data from MLX90614

    return;
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

    (void)rtcore_receive(buf_rx, 4);

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

static int
rtcore_send(const uint8_t *p_buffer, size_t byte_count)
{
    int bytes_sent = send(g_fd_socket, p_buffer, byte_count, 0);
    if (bytes_sent == -1)
    {
        DEBUG("ERROR: Unable to send message: %s (%d)\n",
            __FUNCTION__, strerror(errno), errno);
        gb_is_termination_requested = true;
    }

    return bytes_sent;
}

static int
rtcore_receive(uint8_t *p_buffer, size_t byte_count)
{
    int bytes_received = recv(g_fd_socket, p_buffer, byte_count, 0);
    if (bytes_received == -1)
    {
        DEBUG("ERROR: Unable to receive message: %s (%d)\n",
            __FUNCTION__, strerror(errno), errno);
        gb_is_termination_requested = true;
    }

    return bytes_received;
}

static bool
rtcore_ping(void)
{
    uint8_t buf[32];
    uint32_t buf_len;
    bool b_result = false;

    strcpy(buf, RTCORE_MSG_REQUEST_PING);
    buf_len = strlen(RTCORE_MSG_REQUEST_PING) + 1;

    // Request PING from RTCore app
    if (rtcore_send(buf, buf_len) != -1)
    {
        // Check for PING reply
        int bytes_received = rtcore_receive(buf, 32);
        if (bytes_received != -1)
        {
            if (strncmp(buf, RTCORE_MSG_REPLY_PING, (size_t)bytes_received) == 0)
            {
                // Received correct PING reply
                b_result = true;
            }
        }
    }

    return b_result;
}

static void
display_screen(uint8_t id_screen)
{
    char line_buffer[32];

    u8g2_ClearBuffer(&g_u8g2);

    switch (id_screen)
    {
        case 1:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_tr);
            snprintf(line_buffer, 16, ".. Starting ..");
            u8g2_DrawStr(&g_u8g2, 0, 10, line_buffer);
        break;

        case 2:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_tr);
            snprintf(line_buffer, 16, "Screen 2");
            u8g2_DrawStr(&g_u8g2, 10, 10, line_buffer);
        break;

        default:
        break;
    }

    u8g2_SendBuffer(&g_u8g2);
    return;
}

/* [] END OF FILE */
