/**
 * @file   display.c
 * @brief  This file contains function definitions for the display.
 *
 * @copyright Copyright PND(c) 2022
 *
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "dispcolor.h"
#include "display.h"
#include "gc9a01.h"

#define DISP_REFRESH_TASK_PERIOD            (33)          /*!< DISP application task scheduling rate in ms */

static const char *DBG_TAG = "disp";

// Variables pertaining to screen drawing
static unsigned int disp_tick = 0;

void disp_application_task(void* args);

void DISP_StartDevice(void) {
  // Set default logging level
  esp_log_level_set("disp", ESP_LOG_WARN);

  dispcolor_Init(240, 240);
  dispcolor_ClearScreen();
  dispcolor_Update();
}

void DISP_StartAppTask(void) {
  xTaskCreate(disp_application_task, "display_application_task", 4096, NULL, 5, NULL);
}

/**
 * @brief  disp_application_task draws data to the screen at a set interval
 *
 * @param  args: Not used.
 */
void disp_application_task(void *args) {
  while (1) {
    disp_tick++;
    dispcolor_ClearScreen();
    dispcolor_printf_Bg(((disp_tick * 3) % 240), ((disp_tick * 3) % 240), 1, WHITE, BLACK, "HELLO");
    dispcolor_Update();
    vTaskDelay(pdMS_TO_TICKS(DISP_REFRESH_TASK_PERIOD));
  }
}
