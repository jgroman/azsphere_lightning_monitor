/***************************************************************************//**
* @file    main.h
* @version 1.0.0
* @authors Jaroslav Groman
*
* @par Project Name
*    
*
* @par Description
*    .
*
* @par Target device
*    Azure Sphere MT3620
*
* @par Related hardware
*    Avnet Azure Sphere Starter Kit
*
* @par Code Tested With
*    1. Silicon: Avnet Azure Sphere Starter Kit
*    2. IDE: Visual Studio 2017
*    3. SDK: Azure Sphere SDK Preview
*
* @par Notes
*    .
*
*******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <signal.h>

#include <applibs/gpio.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

#include "build_options.h"

// OLED support library
#include "lib_u8g2.h"

// MLX90614 support library
#include "lib_mlx90614.h"

// LPS22HH support library
#include "lib_lps22hh.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

// !!! Define partner RTCore app Component ID here and also in 
// Project Properties - Debugging - Partner Components
#define PARTNER_RTCORE_COMPONENT_ID "ac067706-9bda-425a-a766-a91a2c4e32c0"

#define RTCORE_MSG_PING             'P'
#define RTCORE_MSG_DATA_REQUEST     'D'

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

/** @brief Application termination handler.
 *
 * Signal handler for termination requests. This handler must be
 * async-signal-safe.
 *
 * @param signal_number
 *
 */
void
handle_termination(int signal_number);

/** @brief Sends message to real-time capable application.
 *
 */
void
handle_request_data(void);

void
handle_rtcore_receive(void);

void
handle_azure_upload(void);

void
handle_button1_press(void);

void
handle_button2_press(void);

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

// Termination state flag
extern volatile sig_atomic_t gb_is_termination_requested;

// File descriptors
extern int g_fd_socket;
extern int g_fd_epoll;
extern int g_fd_i2c;

extern int g_fd_gpio_rgbled_red;    // Red RGB LED GPIO file descriptor
extern int g_fd_gpio_rgbled_green;  // Green RGB LED GPIO file descriptor


extern u8g2_t g_u8g2;           // OLED device descriptor for u8g2
extern mlx90614_t *gp_mlx;      // MLX90614 sensor device descriptor pointer
extern stmdev_ctx_t *gp_lps22hh_ctx;


#ifdef __cplusplus
}
#endif

#endif  // MAIN_H

/* [] END OF FILE */
