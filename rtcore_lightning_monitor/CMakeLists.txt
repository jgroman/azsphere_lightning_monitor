#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

CMAKE_MINIMUM_REQUIRED(VERSION 3.11)
PROJECT(rtcore_lightning_monitor C)

# Create executable
ADD_EXECUTABLE(${PROJECT_NAME} main.c mt3620-intercore.c mt3620-uart-poll.c mt3620-adc.c mt3620-gpio.c mt3620-timer.c mt3620-timer-user.c mt3620-timer-poll.c)
TARGET_LINK_LIBRARIES(${PROJECT_NAME})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/linker.ld)

# Add MakeImage post-build command
INCLUDE("${AZURE_SPHERE_MAKE_IMAGE_FILE}")
