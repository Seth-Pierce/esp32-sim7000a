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

/**
 * @brief  DISP_DeviceStart initializes the display.
 *
 */
void DISP_StartDevice(void);

/**
 * @brief  DISP_StartAppTask starts the display servicer application task.
*/
void DISP_StartAppTask(void);