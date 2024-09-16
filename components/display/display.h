/**
 * @file   display.h
 * @brief  This file contains function declarations for the display.
 *
 * @copyright Copyright PND(c) 2022
 *
 */

#pragma once

#include <stdint.h>
#include <stdio.h>

typedef enum {
    DISP_EV_ACTIVE_CARRIER,
    DISP_EV_CURR_CSQ,
    DISP_EV_SAT_FIX,
    DISP_EV_SAT_IN_VIEW,
    DISP_EV_SAT_IN_USE,
    DISP_EV_SPEED_MPH,
    DISP_EV_COG_DEGREES,
} DisplayEvent_e;

void DISP_OnEvent(DisplayEvent_e event, unsigned int data);

/**
 * @brief  DISP_DeviceStart initializes the display.
 *
 */
void DISP_StartDevice(void);

/**
 * @brief  DISP_StartAppTask starts the display servicer application task.
*/
void DISP_StartAppTask(void);