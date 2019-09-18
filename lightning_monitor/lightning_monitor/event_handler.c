

#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include <sys/socket.h>

#include <applibs/log.h>
#include <applibs/gpio.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

#include "main.h"
#include "event_handler.h"

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

int g_fd_gpio_button1 = -1;
int g_fd_gpio_button2 = -1;

int g_fd_poll_timer_button = -1;
int g_fd_poll_timer_sample = -1;


static GPIO_Value_Type g_state_button1 = GPIO_Value_High;
static GPIO_Value_Type g_state_button2 = GPIO_Value_High;

EventData g_event_data_poll_sample = {          // RTCore Poll Event data
    .eventHandler = &event_handler_timer_sample
};

EventData g_event_data_socket = {      // Socket Event data
    .eventHandler = &event_handler_socket
};

EventData g_event_data_button = {          // Button Event data
    .eventHandler = &event_handler_timer_button
};



/*******************************************************************************
* Function definitions
*******************************************************************************/

void
event_handler_timer_sample(EventData *event_data)
{
    // Collect measurement samples from all sources

    bool b_is_all_ok = true;

    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_sample) != 0) 
    {
        // Failed to consume timer event
        gb_is_termination_requested = true;
        b_is_all_ok = false;
    }

    if (b_is_all_ok)
    {
        // Send measurement sample requests to all data sources
        handle_sample_request();
    }

    return;
}

void
event_handler_timer_button(EventData *event_data)
{
    bool b_is_all_ok = true;
    GPIO_Value_Type state_button1_current;
    GPIO_Value_Type state_button2_current;

    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_button) != 0) 
    {
        // Failed to consume timer event
        gb_is_termination_requested = true;
        b_is_all_ok = false;
    }

    if (b_is_all_ok)
    {
        // Check for a button1 press
        if (GPIO_GetValue(g_fd_gpio_button1, &state_button1_current) != 0)
        {
            Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n",
                strerror(errno), errno);
            gb_is_termination_requested = true;
            b_is_all_ok = false;
        }
        else if (state_button1_current != g_state_button1)
        {
            if (state_button1_current == GPIO_Value_Low)
            {
                handle_button1_press();
            }
            g_state_button1 = state_button1_current;
        }
    }

    if (b_is_all_ok)
    {
        // Check for a button2 press
        if (GPIO_GetValue(g_fd_gpio_button2, &state_button2_current) != 0)
        {
            Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n",
                strerror(errno), errno);
            gb_is_termination_requested = true;
            b_is_all_ok = false;
        }
        else if (state_button2_current != g_state_button2)
        {
            if (state_button2_current == GPIO_Value_Low)
            {
                handle_button2_press();
            }
            g_state_button2 = state_button2_current;
        }
    }

    return;
}


void
event_handler_socket(EventData *eventData)
{
    handle_rtcore_receive();
}

/* [] END OF FILE */
