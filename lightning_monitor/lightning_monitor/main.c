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

// RTCore message buffer size
#define RTCORE_MESSAGE_BUFFER_SIZE  (32u)

// RTCore data reply message length
#define RTCORE_DATA_LENGTH  (16u)

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

typedef struct
{
    float temperature_ambient;

    float temperature_remote;
} mlx90614_data_t;

typedef enum
{
    SCR_INTRO,
    SCR_MAIN,
    SCR_TA7642,
    SCR_MLX90614
} screen_id_t;


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
display_screen(screen_id_t scr_id);

static double
convert_ta7642_out_to_volts(uint32_t ta7642_output);

/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
volatile sig_atomic_t gb_is_termination_requested = false;

int g_fd_socket = -1;       // Socket file descriptor
int g_fd_epoll = -1;        // Epoll file descriptor
int g_fd_i2c = -1;          // I2C interface file descriptor

u8g2_t g_u8g2;              // U8g2 library data descriptor

mlx90614_t *gp_mlx;         // MLX90614 sensor device descriptor pointer

// TA7642 module data received from RTCore application
static detector_data_t g_detector_data;

// MLX90614 temperature data
static mlx90614_data_t g_mlx90614_data;

// Displayed screen id
static screen_id_t g_screen_id = SCR_MLX90614;

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

    // Main program
    if (!gb_is_termination_requested)
    {
        u8g2_ClearDisplay(&g_u8g2);

        DEBUG("Waiting for events.\n", __FUNCTION__);

        // Main program loop
        while (!gb_is_termination_requested)
        {
            // Update OLED display contents
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

    switch (g_screen_id)
    {
        case SCR_MAIN:
            g_screen_id = SCR_TA7642;
        break;

        case SCR_TA7642:
            g_screen_id = SCR_MLX90614;
        break;

        case SCR_MLX90614:
            g_screen_id = SCR_MAIN;
        break;

        default:
            g_screen_id = SCR_MAIN;
        break;
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
    uint8_t byte = RTCORE_MSG_DATA_REQUEST;
    (void)rtcore_send(&byte, 1);

    // Request data from MLX90614
    g_mlx90614_data.temperature_ambient = 
        mlx90614_get_temperature_ambient(gp_mlx);

    g_mlx90614_data.temperature_remote = 
        mlx90614_get_temperature_object1(gp_mlx);


    return;
}

void
handle_rtcore_receive(void)
{
    char buf_rx[RTCORE_MESSAGE_BUFFER_SIZE];

    // Read response from RTCore application.
    int bytes_received = rtcore_receive(buf_rx, RTCORE_MESSAGE_BUFFER_SIZE);

    if (bytes_received > 0)
    {
        // Received data request reply
        if (buf_rx[0] == RTCORE_MSG_DATA_REQUEST)
        {
            // Copy data from receive buffer to g_detector_data struct
            memcpy(&g_detector_data, buf_rx + 1, (size_t)(bytes_received - 1));
        }

    }

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
    bool b_result = false;

    buf[0] = RTCORE_MSG_PING;

    // Request PING from RTCore app
    if (rtcore_send(buf, 1) != -1)
    {
        // Check for PING reply
        int bytes_received = rtcore_receive(buf, 32);
        if (bytes_received != -1)
        {
            if (buf[0] == RTCORE_MSG_PING)
            {
                // Received correct PING reply
                b_result = true;
            }
        }
    }

    return b_result;
}

static void
display_screen(screen_id_t scr_id)
{
    char line_buffer[32];
    detector_data_t *dd = &g_detector_data;

    u8g2_ClearBuffer(&g_u8g2);

    switch (scr_id)
    {
        case SCR_MAIN:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_tr);

            snprintf(line_buffer, 32, "    *** MAIN ***");
            u8g2_DrawStr(&g_u8g2, 0, 8, line_buffer);

            snprintf(line_buffer, 32, "LVL: %d", dd->warning_level);
            u8g2_DrawStr(&g_u8g2, 0, 20, line_buffer);

        break;

        case SCR_TA7642:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_tr);

            snprintf(line_buffer, 32, "   *** TA7642 ***");
            u8g2_DrawStr(&g_u8g2, 0, 8, line_buffer);

            if (dd->warning_level == 0)
            {
                snprintf(line_buffer, 32, "Calibrating PWM: %d", dd->pwm_duty_ctrl);
                u8g2_DrawStr(&g_u8g2, 0, 20, line_buffer);

                snprintf(line_buffer, 32, "OUT: %.3f V", convert_ta7642_out_to_volts(dd->ta7642_output));
                u8g2_DrawStr(&g_u8g2, 0, 31, line_buffer);
            }
            else
            {
                snprintf(line_buffer, 32, "PWM: %d, LVL: %d", dd->pwm_duty_ctrl, dd->warning_level);
                u8g2_DrawStr(&g_u8g2, 0, 20, line_buffer);

                snprintf(line_buffer, 32, "AVG: %.3f V", convert_ta7642_out_to_volts(dd->ta7642_output_avg));
                u8g2_DrawStr(&g_u8g2, 0, 31, line_buffer);

                snprintf(line_buffer, 32, "OUT: %.3f V, CNT: %d", convert_ta7642_out_to_volts(dd->ta7642_output), dd->detections_1sec_m32 / 32);
                u8g2_DrawStr(&g_u8g2, 0, 41, line_buffer);

                snprintf(line_buffer, 32, "IDL: %d", dd->idle_timer_sec);
                u8g2_DrawStr(&g_u8g2, 0, 51, line_buffer);
            }

        break;

        case SCR_MLX90614:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_tr);

            snprintf(line_buffer, 32, " *** MLX90614 ***");
            u8g2_DrawStr(&g_u8g2, 0, 8, line_buffer);

            snprintf(line_buffer, 32, "T-rem: %.1f degC", 
                g_mlx90614_data.temperature_remote);
            u8g2_DrawStr(&g_u8g2, 0, 20, line_buffer);

            snprintf(line_buffer, 32, "T-amb: %.1f degC",
                g_mlx90614_data.temperature_ambient);
            u8g2_DrawStr(&g_u8g2, 0, 31, line_buffer);

        break;

        default:
        break;
    }

    u8g2_SendBuffer(&g_u8g2);
    return;
}

static double
convert_ta7642_out_to_volts(uint32_t ta7642_output)
{
    return (2.5 * ta7642_output) / 4096;
}

/* [] END OF FILE */
