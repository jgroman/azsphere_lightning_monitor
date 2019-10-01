/***************************************************************************//**
* @file    support.h
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

#ifndef SUPPORT_H
#define SUPPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*   Included Headers
*******************************************************************************/

//#include "main.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#ifdef APP_DEBUG
#define DEBUG(s, f, ...) log_printf("%s %s: " s "\n", "App", f, ## __VA_ARGS__)
#define DEBUG_DEV(s, f, d, ...) log_printf("%s %s (0x%02X): " s "\n", "MLX", f,\
                                            d->i2c_addr, ## __VA_ARGS__)
#else
#define DEBUG(s, f, ...)
#define DEBUG_DEV(s, f, d, ...)
#endif // APP_DEBUG

#define ERROR(s, f, ...) log_printf("%s %s: " s "\n", "App", f, \
                                    ## __VA_ARGS__)

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/


/*******************************************************************************
*   Function Declarations
*******************************************************************************/

/**
 * @brief Platform dependent log print function.
 *
 * @param p_format The message string to log.
 * @param ... Argument list.
 *
 * @result 0 for success, or -1 for failure, in which case errno is set
 * to the error value.
 */
int
log_printf(const char *p_format, ...);

/**
 * @brief Close file descriptor.
 *
 * @param fd File descriptor.
 * @param p_fd_name File descriptor name.
 */
void
fd_close(int fd, const char *p_fd_name);

#ifdef __cplusplus
}
#endif

#endif  // SUPPORT_H

/* [] END OF FILE */
