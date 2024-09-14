#pragma once

#include <stdint.h>

typedef enum {
    SIM7000,
} uart_callback_tag_t;

typedef void (*UART_Read_Callback_fp)(const uint8_t *buf, size_t buf_size);

void UART_StartDevice(void);

void UART_WriteData(const uint8_t *data, size_t len);

void UART_SetReadClbk(UART_Read_Callback_fp clbk);