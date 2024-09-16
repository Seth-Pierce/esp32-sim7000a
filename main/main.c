/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "gpio_internal.h"
#include "uart_internal.h"
#include "sim7000a.h"

void app_main(void)
{
    DISP_StartDevice();
    DISP_StartAppTask();
    GPIO_StartDevice();
    UART_StartDevice();
    SIM_StartDevice();
}
