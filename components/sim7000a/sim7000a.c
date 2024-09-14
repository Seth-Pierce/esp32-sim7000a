#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "uart_internal.h"

#define DBG_TAG "SIM"

#define RX_RING_SIZE (1024)
#define RX_WORK_BUF_SIZE (1024)
#define MAX_NUM_CARRIERS (4)

static void sim_task(void *pvParameters);
static void sim_read_callback(const uint8_t *buf, size_t buf_size);
static void sim_parse_fsm(void);

enum AT_Command_e {
    AT_ECHO_TEST,
    AT_DISABLE_COMMAND_ECHO,
    AT_ENABLE_COMMAND_ECHO,
    AT_SET_CHARSET,
    AT_SET_PDU,
    AT_DISABLE_GNSS,
    AT_ENABLE_GNSS,
    AT_LIST_CARRIERS,
    AT_NUM_COMMANDS,
};

enum ARD_Command_e {
    ARD_INIT,
    ARD_SETBAUD,
    ARD_ALIVE,
    ARD_READY,
    ARD_NUM_COMMANDS,
};

enum RuntimeInit_state_e {
    RS_INIT_UNKNOWN,
    RS_INIT_AWAIT_ARD_READY,
    RS_INIT_AWAIT_AT_ECHO,
    RS_INIT_AWAIT_SET_CHARSET_OK,
    RS_INIT_AWAIT_SET_PDU_OK,
    RS_INIT_AWAIT_GNSS_PWR_OK,
    RS_INIT_DONE_PARSE_STATUS,
};

enum RuntimeParse_state_e {
    RS_PARSE_UNKNOWN,
    
    RS_PARSE_REQUEST_CELL_OPERATORS,
    RS_PARSE_AWAIT_CELL_OPERATORS,
    RS_PARSE_POST_CELL_OPERATORS,
    
    RS_PARSE_REQUEST_CURR_CELL_CSQ,
    PS_PARSE_AWAIT_CURR_CELL_CSQ,
    RS_PARSE_POST_CURR_CELL_CSQ,

    RS_PARSE_REQUEST_CURR_NMEA,
    RS_PARSE_AWAIT_CURR_NMEA,
    RS_PARSE_POST_CURR_NMEA,
};

enum CellularAccessTech_e {
    CELL_TECH_UNKNOWN,
    CELL_TECH_M1,
    CELL_TECH_NBIOT,
};

static const char *at_cmds[AT_NUM_COMMANDS] = {
    "AT\r",
    "ATE0\r",
    "ATE1\r",
    "AT+CSCS=\"GSM\"\r",
    "AT+CGDCONT=1,\"IP\",\"hologram\"\r",
    "AT+CGNSPWR=0\r",
    "AT+CGNSPWR=1\r",
    "AT+COPS=?\r",
};

static const char *ard_cmds[ARD_NUM_COMMANDS] = {
    "ARD=INIT",
    "ARD=SETBAUD",
    "ARD=ALIVE",
    "ARD=READY",
};

struct CarrierID_t {
    bool active;
    unsigned int numeric_oper;
    unsigned int last_rssi;
};

static enum RuntimeInit_state_e init_state = RS_INIT_UNKNOWN;
static enum RuntimeInit_state_e prev_init_state = RS_INIT_UNKNOWN;
static enum RuntimeParse_state_e parse_state = RS_PARSE_UNKNOWN;
static enum RuntimeParse_state_e prev_parse_state = RS_PARSE_UNKNOWN;

static struct CarrierID_t found_carriers[MAX_NUM_CARRIERS] = {0};

static char rx_ring[RX_RING_SIZE] = {0};
static char rx_work_buf[RX_WORK_BUF_SIZE] = {0};
static char *rx_ring_w = rx_ring;

void SIM_StartDevice(void) {
    UART_SetReadClbk(sim_read_callback);

    xTaskCreate(sim_task, "sim_task", 2048, NULL, 12, NULL);
}

void sim_task(void *pvParameters) {
    while (true) {
        if (init_state != prev_init_state) {
            ESP_LOGI(DBG_TAG, "init_state changed (%d) -> (%d)", prev_init_state, init_state);
            prev_init_state = init_state;
        }

        switch (init_state) {
            case RS_INIT_UNKNOWN: {
                ESP_LOGI(DBG_TAG, "ARD: Awaiting ARD to come online...");
                init_state = RS_INIT_AWAIT_ARD_READY;
            } break;

            case RS_INIT_AWAIT_ARD_READY: {
                if (strstr(rx_ring, ard_cmds[ARD_READY])) {
                    ESP_LOGI(DBG_TAG, "ARD: Ready!");
                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    UART_WriteData((uint8_t *)at_cmds[AT_DISABLE_COMMAND_ECHO], strlen(at_cmds[AT_DISABLE_COMMAND_ECHO]));

                    init_state = RS_INIT_AWAIT_AT_ECHO;
                }
            } break;

            case RS_INIT_AWAIT_AT_ECHO: {
                if (strstr(rx_ring, "OK")) {
                    ESP_LOGI(DBG_TAG, "S7: Received init echo!");
                    
                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    UART_WriteData((uint8_t *)at_cmds[AT_SET_CHARSET], strlen(at_cmds[AT_SET_CHARSET]));
                    init_state = RS_INIT_AWAIT_SET_CHARSET_OK;
                }
            } break;

            case RS_INIT_AWAIT_SET_CHARSET_OK: {
                if (strstr(rx_ring, "OK")) {
                    ESP_LOGI(DBG_TAG, "S7: Charset set success: %s", at_cmds[AT_SET_CHARSET]);

                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    UART_WriteData((uint8_t *)at_cmds[AT_SET_PDU], strlen(at_cmds[AT_SET_PDU]));
                    init_state = RS_INIT_AWAIT_SET_PDU_OK;
                }
            } break;

            case RS_INIT_AWAIT_SET_PDU_OK: {
                if (strstr(rx_ring, "OK")) {
                    ESP_LOGI(DBG_TAG, "S7: PDU set success: %s", at_cmds[AT_SET_PDU]);

                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    UART_WriteData((uint8_t *)at_cmds[AT_ENABLE_GNSS], strlen(at_cmds[AT_ENABLE_GNSS]));
                    init_state = RS_INIT_AWAIT_GNSS_PWR_OK;
                }
            } break;

            case RS_INIT_AWAIT_GNSS_PWR_OK: {
                if (strstr(rx_ring, "OK")) {
                    ESP_LOGI(DBG_TAG, "S7: GNSS power on success: %s", at_cmds[AT_ENABLE_GNSS]);

                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    init_state = RS_INIT_DONE_PARSE_STATUS;
                }
            } break;

            case RS_INIT_DONE_PARSE_STATUS: {
                sim_parse_fsm();
            } break;

            default: {
                ESP_LOGE(DBG_TAG, "init_state ERROR! DEFAULT!?");    
            } break;
        }

        // if (strlen(rx_ring)) {
        //     ESP_LOGI(DBG_TAG, "[%u]: %s", strlen(rx_ring), rx_ring);
        // }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sim_parse_fsm(void) {
    if (parse_state != prev_parse_state) {
        ESP_LOGI(DBG_TAG, "parse_state changed (%d) -> (%d)", prev_parse_state, parse_state);
        prev_parse_state = parse_state;
    }

    switch (parse_state) {
        case RS_PARSE_UNKNOWN: {
            parse_state = RS_PARSE_REQUEST_CELL_OPERATORS;
        } break;

        case RS_PARSE_REQUEST_CELL_OPERATORS: {
            UART_WriteData((uint8_t *)at_cmds[AT_LIST_CARRIERS], strlen(at_cmds[AT_LIST_CARRIERS]));

            parse_state = RS_PARSE_AWAIT_CELL_OPERATORS;
        } break;
        
        case RS_PARSE_AWAIT_CELL_OPERATORS: {
            if (strstr(rx_ring, "OK") && strstr(rx_ring, "+COPS")) {
                ESP_LOGI(DBG_TAG, "%s", rx_ring);

                if (strlen(rx_ring) < RX_WORK_BUF_SIZE) {
                    memcpy(rx_work_buf, rx_ring, strlen(rx_ring) * sizeof(char));
                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;

                    char *token;
                    char *outer_save_ptr = NULL;
                    char *inner_save_ptr = NULL;

                    const char outer_delims[] = "()\r";
                    const char inner_delims[] = ",\r";

                    token = strtok_r(rx_work_buf, outer_delims, &outer_save_ptr);

                    while (token != NULL) {
                        ESP_LOGI(DBG_TAG, "Outer: %s", token);

                        char *inner_token = strtok_r(token, inner_delims, &inner_save_ptr);

                        while (inner_token != NULL) {
                            ESP_LOGI(DBG_TAG, "Inner: %s", inner_token);
                            inner_token = strtok_r(NULL, inner_delims, &inner_save_ptr);
                        }

                        token = strtok_r(NULL, outer_delims, &outer_save_ptr);
                    }
                }
    

                parse_state = RS_PARSE_POST_CELL_OPERATORS;
            }
        } break;
        
        case RS_PARSE_POST_CELL_OPERATORS: {
        } break;
        
        case RS_PARSE_REQUEST_CURR_CELL_CSQ: {

        } break;
        
        case PS_PARSE_AWAIT_CURR_CELL_CSQ: {

        } break;
        
        case RS_PARSE_POST_CURR_CELL_CSQ: {

        } break;
        
        case RS_PARSE_REQUEST_CURR_NMEA: {

        } break;
        
        case RS_PARSE_AWAIT_CURR_NMEA: {

        } break;
        
        case RS_PARSE_POST_CURR_NMEA: {

        } break;

        default: {
            ESP_LOGE(DBG_TAG, "RS_PARSE IN BAD STATE!!!");
            parse_state = RS_PARSE_UNKNOWN;
        }
    }
}

void sim_read_callback(const uint8_t *buf, size_t buf_size) {
    // ESP_LOGI(DBG_TAG, "[%u]: %s", buf_size, buf);
    while (buf_size) {
        if (*buf) {  // Don't store NULL characters, if they come in!
            *(rx_ring_w++) = *(const char *)(buf++);
            if (rx_ring_w >= &rx_ring[RX_RING_SIZE - 1]) {
                rx_ring_w = rx_ring;
            }

            buf_size--;
        } else {
            buf++;
            buf_size--;
        }
    }
}