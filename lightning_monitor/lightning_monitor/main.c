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
#include <stdlib.h>


#include <applibs/log.h>
#include <applibs/i2c.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// RTCore app connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>

#include "azure_iot_utilities.h"
#include "init.h"
#include "support.h"
#include "main.h"

#include "graphics.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

// RTCore message buffer size
#define RTCORE_MESSAGE_BUFFER_SIZE  (32u)

// RTCore data reply message length
#define RTCORE_DATA_LENGTH          (16u)

// Warning levels for RGB LED
#define WARNING_LEVEL_GREEN         (2000u)
#define WARNING_LEVEL_YELLOW        (4000u)
#define WARNING_LEVEL_RED           (6000u)

// Warning level bar graph extremes
#define WARNING_LEVEL_MAX           (7000u)
#define WARNING_LEVEL_MIN           (255u)

// Atmospheric pressure bar graph extremes
#define PRESSURE_MAX_HPA            (1070u)
#define PRESSURE_MIN_HPA            (920u)

// Cloudiness bar graph extremes
#define CLOUD_DELTA_T_MAX           (10u)
#define CLOUD_DELTA_T_MIN           (0u)

// Maximum line length on OLED display
#define MAX_LINE_LEN                (32U)

#define JSON_BUFFER_SIZE 128

#define STR_SPACE                   " "
#define STR_SENSOR                  "Sensor"
#define STR_NOT_DETECTED            "not detected"
#define STR_DEG_C                   "\xB0""C"
#define STR_REMOTE                  "Remote"
#define STR_PRESSURE                "Pressure"
#define STR_CALIBRATING             "Calibrating"

typedef struct
{
    // PWM duty control value
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

    // Time since the last impulse detection.
    // 2 bytes
    uint16_t idle_timer_sec;

} detector_data_t;

typedef struct
{
    // Ambient (sensor) temperature
    float temperature_ambient;

    // Remote temperature
    float temperature_remote;
} mlx90614_data_t;

typedef enum
{
    SCR_LOGO,
    SCR_MAIN,
    SCR_MAIN_2,
    SCR_TA7642,
    SCR_MLX90614,
    SCR_LPS22HH
} screen_id_t;

const struct timespec SLEEP_TIME_5S = { 5, 0 };

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

static int
rtcore_send(const uint8_t *p_buffer, size_t byte_count);

static int
rtcore_receive(uint8_t *p_buffer, size_t byte_count);

static void
display_screen(screen_id_t scr_id);

static double
convert_ta7642_out_to_volts(uint32_t ta7642_output);

static void
show_level_on_rgb(void);

/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
volatile sig_atomic_t gb_is_termination_requested = false;

int g_fd_socket = -1;       // Socket file descriptor
int g_fd_epoll = -1;        // Epoll file descriptor
int g_fd_i2c = -1;          // I2C interface file descriptor

int g_fd_gpio_rgbled_red = -1;    // Red RGB LED GPIO file descriptor
int g_fd_gpio_rgbled_green = -1;  // Green RGB LED GPIO file descriptor

// U8g2 library data descriptor
u8g2_t g_u8g2;

// MLX90614 sensor device descriptor pointer
mlx90614_t *gp_mlx;

// LPS22HH sensor device descriptor pointer
stmdev_ctx_t *gp_lps22hh_ctx;

// TA7642 module data received from RTCore application
static detector_data_t g_detector_data;

// MLX90614 temperature data
static mlx90614_data_t g_mlx90614_data;

// Displayed screen id
static screen_id_t g_screen_id = SCR_MAIN;

// Display line buffer
static char g_line_buffer[MAX_LINE_LEN];

// LPS22HH raw pressure data
// There are only 3 bytes of data but we will cast it to int32_t later
static uint8_t g_lps_raw_pressure[4];

// LPS22HH calculated pressure value
static float g_lps_pressure_hpa;

// LPS22HH general register
static lps22hh_reg_t g_lps_reg;

// Provide local access to variables in other files
//extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;

/*******************************************************************************
* Public function definitions
*******************************************************************************/

/*******************************************************************************
* Main program entry
*******************************************************************************/
int 
main(int argc, char *argv[])
{
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

        // Display app logo
        display_screen(SCR_LOGO);
        nanosleep(&SLEEP_TIME_5S, NULL);

        // Main program loop
        while (!gb_is_termination_requested)
        {
            // Update OLED display contents
            display_screen(g_screen_id);

            // Update RGB LED depending on warning level
            show_level_on_rgb();

            // Handle timers
            if (WaitForEventAndCallHandler(g_fd_epoll) != 0)
            {
                // Event polling failed
                gb_is_termination_requested = true;
            }

#           ifdef IOT_HUB_APPLICATION
            // Setup the IoT Hub client.
            // Notes:
            // - it is safe to call this function even if the client has already been set up, as in
            //   this case it would have no effect;
            // - a failure to setup the client is a fatal error.
            if (!AzureIoT_SetupClient()) 
            {
                Log_Debug("ERROR: Failed to set up IoT Hub client\n");
                break;
            }

            // AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
            // the flow of data with the Azure IoT Hub
            AzureIoT_DoPeriodicTasks();
#           endif
        }
        DEBUG("Exiting main loop.\n", __FUNCTION__);

        u8g2_ClearDisplay(&g_u8g2);

        // Switch off all RGB LEDs
        GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_High);
        GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_High);
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
    switch (g_screen_id)
    {
        case SCR_MAIN:
            g_screen_id = SCR_MAIN_2;
        break;

        case SCR_MAIN_2:
            g_screen_id = SCR_TA7642;
        break;

        case SCR_TA7642:
            g_screen_id = SCR_MLX90614;
        break;

        case SCR_MLX90614:
            g_screen_id = SCR_LPS22HH;
        break;

        case SCR_LPS22HH:
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

    // Read data from MLX90614
    g_mlx90614_data.temperature_ambient = 
        mlx90614_get_temperature_ambient(gp_mlx);

    g_mlx90614_data.temperature_remote = 
        mlx90614_get_temperature_object1(gp_mlx);

    // Read data from LPS22HH
    // -- Read LPS22HH status
    lps22hh_read_reg(gp_lps22hh_ctx, LPS22HH_STATUS, (uint8_t *)&g_lps_reg, 1);

    // -- Check that new pressure data are ready
    if (g_lps_reg.status.p_da == 1)
    {
        lps22hh_pressure_raw_get(gp_lps22hh_ctx, g_lps_raw_pressure);
        g_lps_raw_pressure[3] = 0;
        g_lps_pressure_hpa =
            lps22hh_from_lsb_to_hpa(*(int32_t *)g_lps_raw_pressure);
    }

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

void
handle_azure_upload(void)
{
#   ifdef IOT_HUB_APPLICATION
    char *p_buffer_json;

    if ((p_buffer_json = malloc(JSON_BUFFER_SIZE)) == NULL)
    {
        ERROR("ERROR: not enough memory for upload buffer.", __FUNCTION__);
    }
    else
    {
        // Construct Azure upload message
        snprintf(p_buffer_json, JSON_BUFFER_SIZE, 
            "{\"warning_level\":\"%d\", \"pressure\":\"%.2f\", "
            "\"temp_delta\":\"%.1f\"}",
            g_detector_data.warning_level, g_lps_pressure_hpa, 
            g_mlx90614_data.temperature_remote - 
            g_mlx90614_data.temperature_ambient);

        DEBUG("Uploading to Azure: %s", __FUNCTION__, p_buffer_json);
        
        //AzureIoT_SendMessage(pjsonBuffer);

        free(p_buffer_json);
    }
#   endif

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

static void
display_screen(screen_id_t scr_id)
{
    detector_data_t *dd = &g_detector_data;
    float bar_length;

    float warning_level_cropped;
    float pressure_cropped;
    float cloud_temp_cropped;
    float cloud_temp_delta;

    const u8g2_uint_t BAR_LENGTH_MAX = 108;

#   define  H_BAR           14
#   define  X_BAR_START     20
#   define  Y_BAR_1         8
#   define  Y_BAR_2         29
#   define  Y_BAR_3         50

    u8g2_ClearBuffer(&g_u8g2);

    switch (scr_id)
    {
        case SCR_LOGO:
            u8g2_DrawXBM(&g_u8g2, 31, 0, XBM_LOGO_W, XBM_LOGO_H, XBM_LOGO_BITS);
        break;

        case SCR_MAIN:
            // Warning level bar graph
            warning_level_cropped = (dd->warning_level < WARNING_LEVEL_MIN) ?
                WARNING_LEVEL_MIN :
                (dd->warning_level > WARNING_LEVEL_MAX) ?
                    WARNING_LEVEL_MAX :
                    dd->warning_level;

            u8g2_DrawXBM(&g_u8g2, 0, Y_BAR_1, XBM_FLASH_W, XBM_FLASH_H, 
                XBM_FLASH_BITS);
            u8g2_DrawRFrame(&g_u8g2, X_BAR_START, Y_BAR_1, BAR_LENGTH_MAX, 
                H_BAR, 2);
            bar_length = (warning_level_cropped - WARNING_LEVEL_MIN) * 
                ((float)BAR_LENGTH_MAX / (WARNING_LEVEL_MAX - WARNING_LEVEL_MIN));
            if (bar_length > 5)
            {
                // Minimum displayed length depends on bar corner radius 
                // This is u8g2 library limitation
                u8g2_DrawRBox(&g_u8g2, X_BAR_START, Y_BAR_1, 
                    (u8g2_uint_t)bar_length, H_BAR, 2);
            }

            // Pressure bar graph
            pressure_cropped = (g_lps_pressure_hpa < PRESSURE_MIN_HPA) ?
                PRESSURE_MIN_HPA :
                (g_lps_pressure_hpa > PRESSURE_MAX_HPA) ?
                    PRESSURE_MAX_HPA :
                    g_lps_pressure_hpa;

            u8g2_DrawXBM(&g_u8g2, 2, Y_BAR_2, XBM_PRESS_W, XBM_PRESS_H, 
                XBM_PRESS_BITS);
            u8g2_DrawRFrame(&g_u8g2, X_BAR_START, Y_BAR_2, 
                (u8g2_uint_t)BAR_LENGTH_MAX, H_BAR, 2);
            bar_length = (pressure_cropped - PRESSURE_MIN_HPA) * 
                ((float)BAR_LENGTH_MAX / (PRESSURE_MAX_HPA - PRESSURE_MIN_HPA));
            if (bar_length > 5)
            {
                // Minimum displayed length depends on bar corner radius 
                // This is u8g2 library limitation
                u8g2_DrawRBox(&g_u8g2, X_BAR_START, Y_BAR_2, 
                    (u8g2_uint_t)bar_length, H_BAR, 2);
            }

            // Clouds bar graph
            cloud_temp_delta = g_mlx90614_data.temperature_remote -
                g_mlx90614_data.temperature_ambient;

            cloud_temp_cropped = (cloud_temp_delta < CLOUD_DELTA_T_MIN) ?
                CLOUD_DELTA_T_MIN :
                (cloud_temp_delta > CLOUD_DELTA_T_MAX) ?
                    CLOUD_DELTA_T_MAX :
                    cloud_temp_delta;

            u8g2_DrawXBM(&g_u8g2, 0, Y_BAR_3 + 2, XBM_CLOUD_W, XBM_CLOUD_H, 
                XBM_CLOUD_BITS);
            u8g2_DrawRFrame(&g_u8g2, X_BAR_START, Y_BAR_3, 
                (u8g2_uint_t)BAR_LENGTH_MAX, H_BAR, 2);
            bar_length = (cloud_temp_cropped - CLOUD_DELTA_T_MIN) * 
                ((float)BAR_LENGTH_MAX / (CLOUD_DELTA_T_MAX - CLOUD_DELTA_T_MIN));
            if (bar_length > 5)
            {
                // Minimum displayed length depends on bar corner radius 
                // This is u8g2 library limitation
                u8g2_DrawRBox(&g_u8g2, X_BAR_START, Y_BAR_3, 
                    (u8g2_uint_t)bar_length, H_BAR, 2);
            }

        break;

        case SCR_MAIN_2:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_14b_tf);
            u8g2_SetDrawColor(&g_u8g2, 1);

            u8g2_DrawXBM(&g_u8g2, 0, Y_BAR_1, XBM_FLASH_W, XBM_FLASH_H, 
                XBM_FLASH_BITS);
            snprintf(g_line_buffer, MAX_LINE_LEN, "%d", dd->warning_level);
            u8g2_DrawStr(&g_u8g2, X_BAR_START, Y_BAR_1 + 12, g_line_buffer);

            u8g2_DrawXBM(&g_u8g2, 2, Y_BAR_2, XBM_PRESS_W, XBM_PRESS_H, 
                XBM_PRESS_BITS);
            snprintf(g_line_buffer, MAX_LINE_LEN, "%.1f hPa", g_lps_pressure_hpa);
            u8g2_DrawStr(&g_u8g2, X_BAR_START, Y_BAR_2 + 12, g_line_buffer);

            cloud_temp_delta = g_mlx90614_data.temperature_remote -
                g_mlx90614_data.temperature_ambient;

            u8g2_DrawXBM(&g_u8g2, 0, Y_BAR_3 + 2, XBM_CLOUD_W, XBM_CLOUD_H, 
                XBM_CLOUD_BITS);
            snprintf(g_line_buffer, MAX_LINE_LEN, "\xBB %.1f " STR_DEG_C, 
                cloud_temp_delta);
            u8g2_DrawStr(&g_u8g2, X_BAR_START, Y_BAR_3 + 12, g_line_buffer);
        break;

        case SCR_TA7642:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_mr);
            u8g2_SetDrawColor(&g_u8g2, 0);

            lib_u8g2_DrawCenteredStr(&g_u8g2, 11, STR_SPACE "TA7642" STR_SPACE);

            u8g2_SetDrawColor(&g_u8g2, 1);

            if (dd->warning_level == 0)
            {
                u8g2_SetFont(&g_u8g2, u8g2_font_t0_14b_tf);

                lib_u8g2_DrawCenteredStr(&g_u8g2, 28, STR_CALIBRATING "...");

                snprintf(g_line_buffer, MAX_LINE_LEN, "PWM: %d", 
                    dd->pwm_duty_ctrl);
                lib_u8g2_DrawCenteredStr(&g_u8g2, 46, g_line_buffer);

                snprintf(g_line_buffer, MAX_LINE_LEN, "OUT: %.3f V", 
                    convert_ta7642_out_to_volts(dd->ta7642_output));
                lib_u8g2_DrawCenteredStr(&g_u8g2, 60, g_line_buffer);
            }
            else
            {
                snprintf(g_line_buffer, MAX_LINE_LEN, "PWM: %d, LVL: %d", 
                    dd->pwm_duty_ctrl, dd->warning_level);
                u8g2_DrawStr(&g_u8g2, 0, 23, g_line_buffer);

                snprintf(g_line_buffer, MAX_LINE_LEN, "AVG: %.3f V", 
                    convert_ta7642_out_to_volts(dd->ta7642_output_avg));
                u8g2_DrawStr(&g_u8g2, 0, 35, g_line_buffer);

                snprintf(g_line_buffer, MAX_LINE_LEN, "OUT: %.3f V, CNT: %d", 
                    convert_ta7642_out_to_volts(dd->ta7642_output), 
                    dd->detections_1sec_m32 / 32);
                u8g2_DrawStr(&g_u8g2, 0, 46, g_line_buffer);

                snprintf(g_line_buffer, MAX_LINE_LEN, "IDL: %d", 
                    dd->idle_timer_sec);
                u8g2_DrawStr(&g_u8g2, 0, 57, g_line_buffer);
            }

        break;

        case SCR_MLX90614:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_mr);
            u8g2_SetDrawColor(&g_u8g2, 0);

            lib_u8g2_DrawCenteredStr(&g_u8g2, 11, STR_SPACE "MLX90614" STR_SPACE);

            u8g2_SetFont(&g_u8g2, u8g2_font_t0_14b_tf);
            u8g2_SetDrawColor(&g_u8g2, 1);

            if (gp_mlx)
            {
                snprintf(g_line_buffer, MAX_LINE_LEN, 
                    STR_REMOTE ": %.1f " STR_DEG_C,
                    g_mlx90614_data.temperature_remote);
                u8g2_DrawStr(&g_u8g2, 0, 32, g_line_buffer);

                snprintf(g_line_buffer, MAX_LINE_LEN, 
                    STR_SENSOR ": %.1f " STR_DEG_C,
                    g_mlx90614_data.temperature_ambient);
                u8g2_DrawStr(&g_u8g2, 0, 50, g_line_buffer);
            }
            else
            {
                lib_u8g2_DrawCenteredStr(&g_u8g2, 32, STR_SENSOR);
                lib_u8g2_DrawCenteredStr(&g_u8g2, 50, STR_NOT_DETECTED);
            }

        break;

        case SCR_LPS22HH:
            u8g2_SetFont(&g_u8g2, u8g2_font_t0_11b_mr);
            u8g2_SetDrawColor(&g_u8g2, 0);

            lib_u8g2_DrawCenteredStr(&g_u8g2, 11, STR_SPACE "LPS22HH" STR_SPACE);

            u8g2_SetFont(&g_u8g2, u8g2_font_t0_14b_tf);
            u8g2_SetDrawColor(&g_u8g2, 1);

            if (gp_lps22hh_ctx)
            {
                lib_u8g2_DrawCenteredStr(&g_u8g2, 28, STR_PRESSURE);

                snprintf(g_line_buffer, MAX_LINE_LEN, "%.1f hPa", 
                    g_lps_pressure_hpa);
                lib_u8g2_DrawCenteredStr(&g_u8g2, 44, g_line_buffer);
            }
            else
            {
                lib_u8g2_DrawCenteredStr(&g_u8g2, 32, STR_SENSOR);
                lib_u8g2_DrawCenteredStr(&g_u8g2, 50, STR_NOT_DETECTED);
            }
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

static void
show_level_on_rgb(void)
{
    const uint16_t HYSTERESIS = 20;

    detector_data_t *dd = &g_detector_data;

    if (dd->warning_level > WARNING_LEVEL_RED + HYSTERESIS)
    {
        // Red On, Green Off
        GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_Low);
        GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_High);
    }
    else if ((dd->warning_level > WARNING_LEVEL_YELLOW + HYSTERESIS) && 
        (dd->warning_level < WARNING_LEVEL_RED - HYSTERESIS))
    {
        // Red On, Green On (= Yellow)
        GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_Low);
        GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_Low);
    }
    else if ((dd->warning_level > WARNING_LEVEL_GREEN + HYSTERESIS) &&
        (dd->warning_level < WARNING_LEVEL_YELLOW - HYSTERESIS))
    {
        // Red Off, Green On
        GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_High);
        GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_Low);
    }
    else if (dd->warning_level < WARNING_LEVEL_GREEN - HYSTERESIS)
    {
        // Red Off, Green Off
        GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_High);
        GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_High);
    }

    return;
}

/* [] END OF FILE */
