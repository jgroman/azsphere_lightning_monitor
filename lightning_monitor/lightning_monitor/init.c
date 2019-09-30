

#include <signal.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <applibs/log.h>
#include <applibs/application.h>

#include "lib_u8g2.h"

#include "event_handler.h"
#include "support.h"
#include "init.h"

int
init_handlers(void)
{
    int result = -1;

    // Termination handler
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = handle_termination;
    result = sigaction(SIGTERM, &action, NULL);
    if (result != 0)
    {
        // Failed to set signal action
        ERROR("ERROR: Unable to set sigaction: %s (%d)\n",
            __FUNCTION__, strerror(errno), errno);
    }

    // Epoll handler
    if (result != -1)
    {
        g_fd_epoll = CreateEpollFd();
        if (g_fd_epoll < 0) 
        {
            // Failed to create epoll
            ERROR("ERROR: Unable to create epoll: %s (%d)\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    // Open connection to real-time capable application.
    g_fd_socket = Application_Socket(PARTNER_RTCORE_COMPONENT_ID);
    if (g_fd_socket == -1)
    {
        // Failed to create socket
        ERROR("ERROR: Unable to create socket: %s (%d)",
            __FUNCTION__, strerror(errno), errno);
        ERROR("RTCore App is not loaded or its component Id is not correct.\n",
            __FUNCTION__);
        result = -1;
    }
    else
    {
        // Set timeout, to handle case where real-time capable application 
        // does not respond.
        static const struct timeval SOCKET_RECEIVE_TIMEOUT = 
            { .tv_sec = 5,.tv_usec = 0 };

        result = setsockopt(g_fd_socket, SOL_SOCKET, SO_RCVTIMEO,
            &SOCKET_RECEIVE_TIMEOUT, sizeof(SOCKET_RECEIVE_TIMEOUT));
        if (result == -1)
        {
            // Failed to set socket timeout
            ERROR("ERROR: Unable to set socket timeout: %s (%d)\n",
                __FUNCTION__, strerror(errno), errno);
        }
    }

    // Register handler for incoming messages from RTCore
    if (result != -1)
    {
        if (RegisterEventHandlerToEpoll(g_fd_epoll, g_fd_socket,
            &g_event_data_socket, EPOLLIN) != 0)
        {
            // Failed to register socket event handler
            ERROR("ERROR: Cannot register socket event handler: %s (%d)\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    // Create poll timer for data request
    if (result != -1)
    {
        static const struct timespec SAMPLE_REQUEST_PERIOD = { 4, 0 };

        g_fd_poll_timer_sample = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &SAMPLE_REQUEST_PERIOD, &g_event_data_poll_sample, EPOLLIN);
        if (g_fd_poll_timer_sample < 0)
        {
            // Failed to create data request poll timer
            ERROR("ERROR: Could not create poll timer: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    // Create poll timer for Azure upload
    if (result != -1)
    {
        static const struct timespec AZURE_UPLOAD_PERIOD = { 5 * 60, 0 };

        g_fd_poll_timer_upload = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &AZURE_UPLOAD_PERIOD, &g_event_data_poll_upload, EPOLLIN);
        if (g_fd_poll_timer_upload < 0)
        {
            // Failed to create Azure upload poll timer
            ERROR("ERROR: Could not create poll timer: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

int
init_peripherals(void)
{
    int result = -1;

    // Initialize I2C
    DEBUG("Initializing I2C", __FUNCTION__);
    g_fd_i2c = I2CMaster_Open(I2C_ISU);
    if (g_fd_i2c < 0) 
    {
        ERROR("ERROR: I2CMaster_Open: %s (%d)\n",
            __FUNCTION__, strerror(errno), errno);
    }
    else
    {
        result = I2CMaster_SetBusSpeed(g_fd_i2c, I2C_BUS_SPEED);
        if (result != 0) 
        {
            ERROR("ERROR: I2CMaster_SetBusSpeed: %s (%d)\n",
                __FUNCTION__, strerror(errno), errno);
        }
        else
        {
            result = I2CMaster_SetTimeout(g_fd_i2c, I2C_TIMEOUT_MS);
            if (result != 0) 
            {
                ERROR("ERROR: I2CMaster_SetTimeout: %s (%d)\n",
                    __FUNCTION__, strerror(errno), errno);
            }
        }
    }

    // Initialize 128x64 SSD1306 OLED
    if (result != -1)
    {
        DEBUG("Initializing OLED display.", __FUNCTION__);
        
        // Set lib_u8g2 I2C interface file descriptor and device address
        lib_u8g2_set_i2c(g_fd_i2c, I2C_ADDR_OLED);

        // Set display type and callbacks
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(&g_u8g2, U8G2_R0, 
            lib_u8g2_byte_i2c, lib_u8g2_custom_cb);

        u8g2_InitDisplay(&g_u8g2);

        u8g2_SetPowerSave(&g_u8g2, 0); // Wake up display
    }

    // Initialize MLX90614 sensor
    if (result != -1)
    {
        DEBUG("Initializing MLX90614", __FUNCTION__);
        gp_mlx = mlx90614_open(g_fd_i2c, MLX90614_I2C_ADDRESS);

        if (!gp_mlx)
        {
            ERROR("ERROR: Could not initialize MLX90614.\n", __FUNCTION__);
            // This is optional component, not aborting init process here
        }
        else
        {
            // Set measurement unit to degrees Celsius
            mlx90614_set_temperature_unit(gp_mlx, MLX_TEMP_CELSIUS);
        }
    }

    // Initialize LPS22HH sensor
    if (result != -1)
    {
        DEBUG("Initializing LPS22HH", __FUNCTION__);
        gp_lps22hh_ctx = lps22hh_open_via_lsm6dso(g_fd_i2c);

        if (!gp_lps22hh_ctx)
        {
            ERROR("ERROR: Failed to initialize LPS22HH sensor\n", __FUNCTION__);
            // This is optional component, not aborting init process here
        }
    }

    // Initialize development kit GPIOs
    // -- Open button1 GPIO as input
    if (result != -1)
    {
        DEBUG("Opening PROJECT_BUTTON_1 as input.", __FUNCTION__);
        g_fd_gpio_button1 = GPIO_OpenAsInput(PROJECT_BUTTON_1);
        if (g_fd_gpio_button1 < 0) {
            // Failed to open button1 GPIO
            ERROR("ERROR: Could not open button GPIO: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    // -- Open button2 GPIO as input
    if (result != -1)
    {
        DEBUG("Opening PROJECT_BUTTON_2 as input.", __FUNCTION__);
        g_fd_gpio_button2 = GPIO_OpenAsInput(PROJECT_BUTTON_2);
        if (g_fd_gpio_button2 < 0) {
            // Failed to open button2 GPIO
            ERROR("ERROR: Could not open button GPIO: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    // -- Open Red RGB LED GPIO as output
    if (result != -1)
    {
        DEBUG("Opening PROJECT_RGBLED_RED as output.", __FUNCTION__);
        g_fd_gpio_rgbled_red = GPIO_OpenAsOutput(PROJECT_RGBLED_RED, 
            GPIO_OutputMode_PushPull, GPIO_Value_High);     // LED is Off
        if (g_fd_gpio_rgbled_red < 0)
        {
            // Failed to open Red RGB LED GPIO
            ERROR("ERROR: Could not open Red RGB LED GPIO: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
        else
        {
            // Set Red RGB LED as Off
            GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_High);
        }
    }

    // -- Open Green RGB LED GPIO as output
    if (result != -1)
    {
        DEBUG("Opening PROJECT_RGBLED_GREEN as output.", __FUNCTION__);
        g_fd_gpio_rgbled_green = GPIO_OpenAsOutput(PROJECT_RGBLED_GREEN,
            GPIO_OutputMode_PushPull, GPIO_Value_High);     // LED is Off
        if (g_fd_gpio_rgbled_green < 0)
        {
            // Failed to open Green RGB LED GPIO
            ERROR("ERROR: Could not open Green RGB LED GPIO: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
        else
        {
            // Set Green RGB LED as Off
            GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_High);
        }
    }

    // Create poll timer for button press check
    if (result != -1)
    {
        const struct timespec PERIOD_BTN_PRESS_POLL = { 0, 1000000 };

        g_fd_poll_timer_button = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &PERIOD_BTN_PRESS_POLL, &g_event_data_button, EPOLLIN);
        if (g_fd_poll_timer_button < 0)
        {
            // Failed to create button poll timer
            ERROR("ERROR: Could not create button poll timer: %s (%d).\n",
                __FUNCTION__, strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

void
init_shutdown(void)
{
    // Close Epoll fd
    fd_close(g_fd_epoll, "Epoll");

    // Close LPS22HH sensor
    if (gp_lps22hh_ctx)
    {
        lps22hh_close(gp_lps22hh_ctx);
    }

    // Close MLX90614 sensor
    if (gp_mlx)
    {
        mlx90614_close(gp_mlx);
    }

    // Close I2C
    fd_close(g_fd_i2c, "I2C");

    // Switch off LEDs
    GPIO_SetValue(g_fd_gpio_rgbled_red, GPIO_Value_High);
    GPIO_SetValue(g_fd_gpio_rgbled_green, GPIO_Value_High);

    // Close GPIO fds
    fd_close(g_fd_gpio_button1, "Button1 GPIO");
    fd_close(g_fd_gpio_button2, "Button2 GPIO");
    fd_close(g_fd_gpio_rgbled_red, "Red RGB LED GPIO");
    fd_close(g_fd_gpio_rgbled_green, "Green RGB LED GPIO");
}

/* [] END OF FILE */
