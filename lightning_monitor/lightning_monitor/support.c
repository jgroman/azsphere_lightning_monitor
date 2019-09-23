/***************************************************************************//**
* @file    support.c
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

#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <applibs/log.h>

#include "support.h"

/*******************************************************************************
* Public function definitions
*******************************************************************************/

int
log_printf(const char *p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    int result = Log_DebugVarArgs(p_format, args);
    va_end(args);

    return result;
}

void
fd_close(int fd, const char *p_fd_name)
{
    if (fd >= 0) 
    {
        if (close(fd) != 0)
        {
            // Failed to close file descriptor
            ERROR("ERROR: Could not close file descriptor %s: %s (%d).\n",
                __FUNCTION__, p_fd_name, strerror(errno), errno);
        }
    }
    return;
}

/* [] END OF FILE */
