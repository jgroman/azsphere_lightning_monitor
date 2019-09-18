/*
 * Extension of "official" mt3620-timer.h
 *
 * Parts of code Copyright (c) 2019 Microsoft Corporation. All rights reserved.
 * Copyright (c) 2019 Jaroslav Groman
 *
 * Licensed under the MIT License.
 */


#ifndef MT3620_TIMER_USER_H
#define MT3620_TIMER_USER_H

#include <stdint.h>

#include "mt3620-baremetal.h"


/// <summary>
/// <para>Register a callback for the supplied timer. Only one callback can be registered
/// at a time for each timer. If a callback is already registered, then the timer is
/// cancelled, the new callback is installed, and the timer is restarted. The callback
/// runs in interrupt context.</para>
/// <para>The callback will be invoked once. The callback can re-register itself by calling
/// this function.</para>
/// <para>Only call this function from the main application thread or from a timer callback.</para>
/// <para>The application should install the <see cref="Gpt_HandleIrq1" /> interrupt handler
/// and call <see cref="Gpt_Init" /> before calling this function.</para>
/// </summary>
/// <param name="gpt">Which hardware timer to use.</param>
/// <param name="counter">Count of 1/32 second ticks.</param>
/// <param name="callback">Function to invoke in interrupt context when the timer expires.</param>
void Gpt_LaunchTimer32k(TimerGpt gpt, uint32_t counter, Callback callback);

/// <summary>
///     Start freerunning GPT2 timer.
/// </summary>
/// <param name="initial_value">Initial timer value.</param>
/// <param name="speed_flag">Timer speed unit: 0 ~ 1 kHz, 1 ~ 32 kHz.</param>
void Gpt2_LaunchTimer(uint32_t initial_value, bool speed_flag);

/// <summary>
///     Read current GPT2 counter value.
///     <para>The first value is available after 3T clock after enabling.</para>
/// </summary>
uint32_t Gpt2_GetValue(void);

/// <summary>
///     Blocking millisecond wait loop using GPT2.
/// </summary>
/// <param name="milliseconds">Waiting time in milliseconds.</param>
void Gpt2_WaitMs(uint32_t milliseconds);

#endif /* MT3620_TIMER_USER_H */
