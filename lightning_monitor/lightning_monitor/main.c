/**
 * Copyright (c) 2019 Jaroslav Groman
 *
 * Licensed under the MIT License.
 */

#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>

#include <applibs/log.h>
#include <applibs/gpio.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

// RTCore app connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>

/*******************************************************************************
* #defines adn constants
*******************************************************************************/

// !!! Define partner RTCore app Component ID here and also in 
// Project Properties - Debugging - Partner Components
#define PARTNER_RTCORE_COMPONENT_ID "ac067706-9bda-425a-a766-a91a2c4e32c0"


/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/** @brief Application termination handler.
 *
 * Signal handler for termination requests. This handler must be
 * async-signal-safe.
 *
 * @param signal_number
 *
 */
static void
termination_handler(int signal_number);

/** @brief Timer event handler for polling ADC
 *
 */
static void
adc_timer_event_handler(EventData *event_data);

/** @brief Handle socket event by reading incoming data from real-time capable application.
 *
 */
static void
socket_event_handler(EventData *eventData);

/** @brief Sends message to real-time capable application.
 *
 */
static void
send_msg_to_rtcore(void);

/** @brief Initialize signal handlers.
 *
 * Set up SIGTERM termination handler.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_handlers(void);

/** @brief Initialize peripherals.
 *
 * Initialize all peripherals used by this project.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_peripherals(void);

/**
 *
 */
static void
close_peripherals_and_handlers(void);

/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
static volatile sig_atomic_t gb_is_termination_requested = false;

static int epoll_fd = -1;
static int sock_fd = -1;

static int adc_poll_timer_fd = -1;
static EventData adc_event_data = {          // ADC Event handler data
    .eventHandler = &adc_timer_event_handler
};

static EventData socket_event_data = {      // Socket Event handler data
    .eventHandler = &socket_event_handler
};

/*******************************************************************************
* Main program entry
*******************************************************************************/
int 
main(void)
{
    Log_Debug("*** App starting\n");

    // Initialize handlers
    if (init_handlers() != 0)
    {
        gb_is_termination_requested = true;
    }

    // Initialize peripherals
    if (!gb_is_termination_requested)
    {
        if (init_peripherals() != 0)
        {
            gb_is_termination_requested = true;
        }
    }

    // Main program
    if (!gb_is_termination_requested)
    {
        Log_Debug("Waiting for event\n");

        // Main program loop
        while (!gb_is_termination_requested)
        {
            // Handle timers
            if (WaitForEventAndCallHandler(epoll_fd) != 0)
            {
                gb_is_termination_requested = true;
            }
        }
        Log_Debug("Not waiting for event anymore\n");
    }

    // Clean up and shutdown
    close_peripherals_and_handlers();
}

/*******************************************************************************
* Private functions
*******************************************************************************/
static void
adc_timer_event_handler(EventData *event_data)
{
    // Consume timer event
    if (ConsumeTimerFdEvent(adc_poll_timer_fd) != 0) {
        gb_is_termination_requested = true;
        return;
    }

    // Request ADC data
    Log_Debug("ADC check\n");

    send_msg_to_rtcore();
}

static void 
send_msg_to_rtcore(void)
{
    static int iter = 0;

    // Send "Read-ADC-%d" message to real-time capable application.
    static char txMessage[32];
    sprintf(txMessage, "Read-ADC-%d", iter++);
    Log_Debug("Sending: %s\n", txMessage);

    int bytesSent = send(sock_fd, txMessage, strlen(txMessage), 0);
    if (bytesSent == -1)
    {
        Log_Debug("ERROR: Unable to send message: %d (%s)\n", 
            errno, strerror(errno));
        gb_is_termination_requested = true;
        return;
    }
}

static void 
socket_event_handler(EventData *eventData)
{
    // Read response from real-time capable application.
    char rxBuf[32];
    union Analog_data
    {
        uint32_t u32;
        uint8_t u8[4];
    } analog_data;

    int bytesReceived = recv(sock_fd, rxBuf, sizeof(rxBuf), 0);

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
        analog_data.u8[i] = rxBuf[i];
    }

    // Voltage in Volts = 2.5 * adc_reading / 4096
    double voltage = (2.5 * analog_data.u32) / 4096;

    Log_Debug("Measured voltage: %f V\n", voltage);
}

static void
termination_handler(int signal_number)
{
    gb_is_termination_requested = true;
}

static int
init_handlers(void)
{
    int result = -1;

    // Termination handler
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = termination_handler;
    result = sigaction(SIGTERM, &action, NULL);
    if (result != 0) 
    {
        Log_Debug("ERROR: %s - sigaction: errno=%d (%s)\n",
            __FUNCTION__, errno, strerror(errno));
    }

    // Epoll handler
    if (result != -1)
    {
        epoll_fd = CreateEpollFd();
        if (epoll_fd < 0) {
            result = -1;
        }
    }

    // Open connection to real-time capable application.
    sock_fd = Application_Socket(PARTNER_RTCORE_COMPONENT_ID);
    if (sock_fd == -1)
    {
        Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
        Log_Debug("Real Time Core disabled or Component Id is not correct.\n");
        result = -1;
    }
    else
    {
        // Set timeout, to handle case where real-time capable application 
        // does not respond.
        static const struct timeval recvTimeout = { .tv_sec = 5,.tv_usec = 0 };
        result = setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, 
                    &recvTimeout, sizeof(recvTimeout));
        if (result == -1)
        {
            Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n", 
                errno, strerror(errno));
        }
    }

    // Register handler for incoming messages from RTCore
    if (result != -1)
    {
        if (RegisterEventHandlerToEpoll(epoll_fd, sock_fd, 
                &socket_event_data, EPOLLIN) != 0)
        {
            Log_Debug("ERROR: Cannot register receive event handler: %d (%s)\n",
                errno, strerror(errno));
            result = -1;
        }
    }

    // Create timer for ADC check
    if (result != -1)
    {
        static const struct timespec adc_check_period = { 4, 0 };
        adc_poll_timer_fd = CreateTimerFdAndAddToEpoll(epoll_fd,
            &adc_check_period, &adc_event_data, EPOLLIN);
        if (adc_poll_timer_fd < 0)
        {
            Log_Debug("ERROR: Could not create ADC poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

static int
init_peripherals(void)
{
    int result = 0;

    return result;
}

static void
close_peripherals_and_handlers(void)
{
    // Close Epoll fd
    CloseFdAndPrintError(epoll_fd, "Epoll");
}

/* [] END OF FILE */
