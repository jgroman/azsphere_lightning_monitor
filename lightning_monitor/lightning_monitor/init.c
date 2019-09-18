

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
        Log_Debug("ERROR: %s - sigaction: errno=%d (%s)\n",
            __FUNCTION__, errno, strerror(errno));
    }

    // Epoll handler
    if (result != -1)
    {
        g_fd_epoll = CreateEpollFd();
        if (g_fd_epoll < 0) {
            result = -1;
        }
    }

    // Open connection to real-time capable application.
    g_fd_socket = Application_Socket(PARTNER_RTCORE_COMPONENT_ID);
    if (g_fd_socket == -1)
    {
        Log_Debug("ERROR: Unable to create socket: %d (%s)\n",
            errno, strerror(errno));
        Log_Debug("Real Time Core disabled or Component Id is not correct.\n");
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
            Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n",
                errno, strerror(errno));
        }
    }

    // Register handler for incoming messages from RTCore
    if (result != -1)
    {
        if (RegisterEventHandlerToEpoll(g_fd_epoll, g_fd_socket,
            &g_event_data_socket, EPOLLIN) != 0)
        {
            Log_Debug("ERROR: Cannot register receive event handler: %d (%s)\n",
                errno, strerror(errno));
            result = -1;
        }
    }

    // Create timer for sample poll
    if (result != -1)
    {
        static const struct timespec SAMPLE_REQUEST_PERIOD = { 4, 0 };

        g_fd_poll_timer_sample = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &SAMPLE_REQUEST_PERIOD, &g_event_data_poll_sample, EPOLLIN);
        if (g_fd_poll_timer_sample < 0)
        {
            Log_Debug("ERROR: Could not create sample poll timer: %s (%d).\n",
                strerror(errno), errno);
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
    Log_Debug("Init I2C\n");
    g_fd_i2c = I2CMaster_Open(I2C_ISU);
    if (g_fd_i2c < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n",
            errno, strerror(errno));
    }
    else
    {
        result = I2CMaster_SetBusSpeed(g_fd_i2c, I2C_BUS_SPEED);
        if (result != 0) {
            Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n",
                errno, strerror(errno));
        }
        else
        {
            result = I2CMaster_SetTimeout(g_fd_i2c, I2C_TIMEOUT_MS);
            if (result != 0) {
                Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n",
                    errno, strerror(errno));
            }
        }
    }

    // Initialize 128x64 SSD1306 OLED
    if (result != -1)
    {
        Log_Debug("Initializig OLED display.\n");
        // Set u8x8 display type and callbacks
        u8x8_Setup(&g_u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c,
            u8x8_byte_i2c, lib_u8g2_custom_cb);
        // Set u8x8 I2C address
        u8x8_SetI2CAddress(&g_u8x8, I2C_ADDR_OLED);
        lib_u8g2_set_i2c_fd(g_fd_i2c);
    }

    // Initialize development kit button GPIO
    // -- Open button1 GPIO as input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_BUTTON_1 as input.\n");
        g_fd_gpio_button1 = GPIO_OpenAsInput(PROJECT_BUTTON_1);
        if (g_fd_gpio_button1 < 0) {
            Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // -- Open button2 GPIO as input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_BUTTON_2 as input.\n");
        g_fd_gpio_button2 = GPIO_OpenAsInput(PROJECT_BUTTON_2);
        if (g_fd_gpio_button2 < 0) {
            Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Create timer for button press check
    if (result != -1)
    {
        struct timespec button_press_check_period = { 0, 1000000 };

        g_fd_poll_timer_button = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &button_press_check_period, &g_event_data_button, EPOLLIN);
        if (g_fd_poll_timer_button < 0)
        {
            Log_Debug("ERROR: Could not create button poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }


    return result;
}

void
init_shutdown(void)
{
    // Close Epoll fd
    CloseFdAndPrintError(g_fd_epoll, "Epoll");

    // Close I2C
    CloseFdAndPrintError(g_fd_i2c, "I2C");

    // Close button1 GPIO fd
    CloseFdAndPrintError(g_fd_gpio_button1, "Button1 GPIO");
}

/* [] END OF FILE */
