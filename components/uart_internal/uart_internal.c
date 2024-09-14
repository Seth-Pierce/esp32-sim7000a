#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "uart_internal.h"

#define UART_NUM UART_NUM_1
#define TXD_PIN (0)
#define RXD_PIN (1)
#define UART_RX_BUF_SIZE (128)

static const char *DBG_TAG = "UART";

static QueueHandle_t uart1_queue;
static uint8_t rx_buf[UART_RX_BUF_SIZE] = {0};
static UART_Read_Callback_fp read_callback = NULL;

static void uart_event_task(void *pvParameters);
static void uart_tx_task(void *pvParameters);

void UART_StartDevice(void) {
    ESP_LOGI(DBG_TAG, "Beginning UART config");
    esp_err_t err = ESP_OK;

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    err = uart_driver_install(UART_NUM, UART_RX_BUF_SIZE * 2, UART_RX_BUF_SIZE * 2, 20, &uart1_queue, 0);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to install UART driver: %d", err);
        return;
    }

    err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to config UART: %d", err);
        return;
    }

    err = uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to set UART pins: %d", err);
        return;
    }

    ESP_LOGI(DBG_TAG, "Starting UART task");

    const char *data = "AT+CGDCONT=1,\"IP\",\"hologram\"\r";
    uart_write_bytes(UART_NUM, data, strlen(data));

    vTaskDelay(pdMS_TO_TICKS(100));

    const char *data2 = "AT+CSCS=\"GSM\"\r";
    uart_write_bytes(UART_NUM, data2, strlen(data2));

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 12, NULL);
}

// static void uart_event_task(void *pvParameters)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(UART_RX_BUF_SIZE + 1);
//     while (1) {
//         const int rxBytes = uart_read_bytes(UART_NUM, data, UART_RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
//     vTaskDelete(NULL);
// }

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(rx_buf, UART_RX_BUF_SIZE);
            ESP_LOGD(DBG_TAG, "uart[%d] event:", UART_NUM);
            switch (event.type) {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                uart_read_bytes(UART_NUM, rx_buf, event.size, portMAX_DELAY);
                ESP_LOGD(DBG_TAG, "[UART DATA]: %d, %s", event.size, rx_buf);

                if (read_callback) {
                    read_callback(rx_buf, event.size);
                }

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(DBG_TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(DBG_TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(DBG_TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(DBG_TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(DBG_TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                ESP_LOGI(DBG_TAG, "Pattern detection not supported");
                break;
            //Others
            default:
                ESP_LOGI(DBG_TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void uart_tx_task(void *pvParameters) {
    while (1) {
        // const char *data = "AT+COPS=?\r";
        vTaskDelay(pdMS_TO_TICKS(10000));
        // uart_write_bytes(UART_NUM, data, strlen(data));
    }
}

void UART_WriteData(const uint8_t *data, size_t len) {
    uart_write_bytes(UART_NUM, data, len);
}

void UART_SetReadClbk(UART_Read_Callback_fp clbk) {
    read_callback = clbk;
}