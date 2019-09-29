
#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*   Included Headers
*******************************************************************************/

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

extern int g_fd_gpio_button1;
extern int g_fd_gpio_button2;

extern int g_fd_poll_timer_button;
extern int g_fd_poll_timer_sample;
extern int g_fd_poll_timer_upload;

extern EventData g_event_data_poll_sample;
extern EventData g_event_data_poll_upload;
extern EventData g_event_data_socket;
extern EventData g_event_data_button;

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

/**
 * @brief Poll timer event handler for requesting measurements
 */
void
event_handler_timer_sample(EventData *event_data);

/**
 * @brief Poll timer event handler for uploading to Azure
 */
void
event_handler_timer_upload(EventData *event_data);

/**
 * @brief Poll timer event handler for checking button state
 */
void
event_handler_timer_button(EventData *event_data);

/**
 * @brief Handle socket event by reading incoming data from real-time 
 * capable application.
 */
void
event_handler_socket(EventData *eventData);

#ifdef __cplusplus
}
#endif

#endif  // EVENT_HANDLER_H

/* [] END OF FILE */
