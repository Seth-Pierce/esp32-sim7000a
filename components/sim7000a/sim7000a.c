#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "display.h"
#include "uart_internal.h"
#include "gpio_internal.h"
#include "sim7000a.h"

#define DBG_TAG "SIM"

#define RX_RING_SIZE (1024)
#define RX_WORK_BUF_SIZE (1024)
#define MAX_NUM_CARRIERS (10)

static void sim_task(void *pvParameters);
static void sim_read_callback(const uint8_t *buf, size_t buf_size);
static void sim_parse_fsm(void);
static char *find_char(char *str, char c, unsigned int instance);

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

    RS_PARSE_REQUEST_CURRENT_OPER,
    RS_PARSE_AWAIT_CURRENT_OPER,
    RS_PARSE_POST_CURRENT_OPER,
    
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

enum CellularStat_e {
    CELL_STAT_UNKNOWN,
    CELL_STAT_AVAILABLE,
    CELL_STAT_CURRENT,
    CELL_STAT_FORBIDDEN,
};

enum AT_Command_e {
    AT_ECHO_TEST,
    AT_DISABLE_COMMAND_ECHO,
    AT_ENABLE_COMMAND_ECHO,
    AT_SET_CHARSET,
    AT_SET_PDU,
    AT_DISABLE_GNSS,
    AT_ENABLE_GNSS,
    AT_REQUEST_NMEA_STRING,
    AT_LIST_CARRIERS,
    AT_GET_CURRENT_CARRIER,
    AT_GET_CSQ,
    AT_NUM_COMMANDS,
};

static const char *at_cmds[AT_NUM_COMMANDS] = {
    "AT\r",
    "ATE0\r",
    "ATE1\r",
    "AT+CSCS=\"GSM\"\r",
    "AT+CGDCONT=1,\"IP\",\"hologram\"\r",
    "AT+CGNSPWR=0\r",
    "AT+CGNSPWR=1\r",
    "AT+CGNSINF\r",
    "AT+COPS=?\r",
    "AT+COPS?\r",
    "AT+CSQ\r",
};

static const char *ard_cmds[ARD_NUM_COMMANDS] = {
    "ARD=INIT",
    "ARD=SETBAUD",
    "ARD=ALIVE",
    "ARD=READY",
};

struct CarrierID_t {
    bool available;
    enum CellularStat_e stat;
    unsigned int numeric_oper;
    unsigned int last_rssi;
};

struct GnssFix_t {
    bool gnss_on;
    bool fix_acquired;
    long long int utc_date_time;
    int lat;
    int lon;
    int alt;
    int sog_kmh;
    int sog_mph;
    int cog_deg;
    int year;
    int month;
    int day;
    int hrs;
    int mins;
    int secs;
    int millis;
    int sats_in_view;
    int sats_in_use;
};

static enum RuntimeInit_state_e init_state = RS_INIT_UNKNOWN;
static enum RuntimeInit_state_e prev_init_state = RS_INIT_UNKNOWN;
static enum RuntimeParse_state_e parse_state = RS_PARSE_UNKNOWN;
static enum RuntimeParse_state_e prev_parse_state = RS_PARSE_UNKNOWN;

static struct CarrierID_t found_carriers[MAX_NUM_CARRIERS] = {0};
static struct GnssFix_t curr_fix = {0};

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
                }
    

                parse_state = RS_PARSE_POST_CELL_OPERATORS;
            }
        } break;
        
        case RS_PARSE_POST_CELL_OPERATORS: {
            char *token;
            char *outer_save_ptr = NULL;
            char *inner_save_ptr = NULL;

            unsigned int found_oper_codes[MAX_NUM_CARRIERS] = {0};
            unsigned int found_oper_codes_stat[MAX_NUM_CARRIERS] = {0};
            unsigned int *found_oper_codes_ptr = found_oper_codes;
            unsigned int *found_oper_codes_stat_ptr = found_oper_codes_stat;

            const char outer_delims[] = "\r";
            const char inner_delims[] = "()+COPS:\r";

            token = strtok_r(rx_work_buf, outer_delims, &outer_save_ptr);

            while (token != NULL) {
                ESP_LOGD(DBG_TAG, "Outer: %s", token);

                char *inner_token = strtok_r(token, inner_delims, &inner_save_ptr);

                while (inner_token != NULL) {
                    ESP_LOGD(DBG_TAG, "Inner: %s", inner_token);

                    if (isdigit((unsigned char)*inner_token) && (*inner_token - 0x30) != 0) {  // Available carriers will have >0 as their first tag
                        ESP_LOGD(DBG_TAG, "Carrier string: %s", inner_token);

                        char *operator_str = find_char(inner_token, ',', 3);
                        if (operator_str) {
                            if (found_oper_codes_ptr < &found_oper_codes[MAX_NUM_CARRIERS]) {
                                *(found_oper_codes_ptr++) = atoi(++operator_str);
                                *(found_oper_codes_stat_ptr++) = atoi(inner_token);  // gets operator status as it's first key
                            } else {
                                ESP_LOGW(DBG_TAG, "Out of oper_code space!");
                            }
                        }
                    }
                    inner_token = strtok_r(NULL, inner_delims, &inner_save_ptr);
                }
                token = strtok_r(NULL, outer_delims, &outer_save_ptr);
            }

            // Update found_carriers with new data
            ESP_LOGI(DBG_TAG, "Operators found:");
            unsigned int *oper_stat = found_oper_codes_stat;
            for (unsigned int *oper = found_oper_codes; (oper < &found_oper_codes[MAX_NUM_CARRIERS]) && (*oper != 0); oper++, oper_stat++) {
                ESP_LOGI(DBG_TAG, "\t<OPER, STAT>: <%d, %d>", *oper, *oper_stat);
            }

            // Scan to see if we've seen this operator before
            for (struct CarrierID_t *carriers = found_carriers; carriers < &found_carriers[MAX_NUM_CARRIERS]; carriers++) {
                carriers->available = false;

                oper_stat = found_oper_codes_stat;
                for (unsigned int *oper = found_oper_codes; oper < &found_oper_codes[MAX_NUM_CARRIERS]; oper++, oper_stat++) {
                    if ((*oper) && (*oper == carriers->numeric_oper)) {
                        ESP_LOGI(DBG_TAG, "Found existing operator (%d)", *oper);
                        carriers->available = true;
                        carriers->stat = *oper_stat;
                        *oper = 0;  // 0 as we've already handled it
                        break;
                    }
                }
            }

            // Load new carriers
            for (struct CarrierID_t *carriers = found_carriers; carriers < &found_carriers[MAX_NUM_CARRIERS]; carriers++) {
                if (carriers->available == false) {
                    oper_stat = found_oper_codes_stat;
                    for (unsigned int *oper = found_oper_codes; oper < &found_oper_codes[MAX_NUM_CARRIERS]; oper++, oper_stat++) {
                        if (*oper) {
                            ESP_LOGI(DBG_TAG, "Storing operator: %d", *oper);
                            carriers->available = true;
                            carriers->numeric_oper = *oper;
                            carriers->stat = *oper_stat;
                            *oper = 0;
                            break;
                        }
                    }
                }
            }

            for (struct CarrierID_t *carriers = found_carriers; carriers < &found_carriers[MAX_NUM_CARRIERS]; carriers++) {
                if (carriers->available) {
                    ESP_LOGI(DBG_TAG, "Available carrier <OPER, STAT, RSSI>: <%d, %d, %d>", carriers->numeric_oper, carriers->stat, carriers->last_rssi);

                    // Update carrier
                    if (carriers->stat == CELL_STAT_CURRENT) {
                        switch (carriers->numeric_oper) {
                            case 310260: {
                                DISP_OnEvent(DISP_EV_ACTIVE_CARRIER, SIM_CARRIER_TMO);
                            } break;

                            case 311480: {
                                DISP_OnEvent(DISP_EV_ACTIVE_CARRIER, SIM_CARRIER_VZW);
                            } break;

                            case 310410: {
                                DISP_OnEvent(DISP_EV_ACTIVE_CARRIER, SIM_CARRIER_ATT);
                            } break;

                            case 313100: {
                                DISP_OnEvent(DISP_EV_ACTIVE_CARRIER, SIM_CARRIER_ATT);  // Firstnet
                            } break;
                            
                            default: {
                                DISP_OnEvent(DISP_EV_ACTIVE_CARRIER, SIM_CARRIER_UNKNOWN);
                            }
                        }
                    }
                }
            }

            memset(rx_work_buf, 0, sizeof(rx_work_buf));
            parse_state = RS_PARSE_REQUEST_CURR_CELL_CSQ;
        } break;

        // case RS_PARSE_REQUEST_CURRENT_OPER: {
        //     UART_WriteData((uint8_t *)at_cmds[AT_GET_CURRENT_CARRIER], strlen(at_cmds[AT_GET_CURRENT_CARRIER]));

        //     parse_state = RS_PARSE_AWAIT_CURRENT_OPER;
        // } break;

        // case RS_PARSE_AWAIT_CURRENT_OPER: {
        //     if (strstr(rx_ring, "OK")) {
        //         ESP_LOGI(DBG_TAG, "%s", rx_ring);

        //         if (strlen(rx_ring) < RX_WORK_BUF_SIZE) {
        //             memcpy(rx_work_buf, rx_ring, strlen(rx_ring) * sizeof(char));
        //             memset(rx_ring, 0, sizeof(rx_ring));
        //             rx_ring_w = rx_ring;
        //         }
    
        //         ESP_LOGI(DBG_TAG, "Current oper str: %s", rx_work_buf);

        //         parse_state = RS_PARSE_POST_CURRENT_OPER;
        //     }
        // } break;

        // case RS_PARSE_POST_CURRENT_OPER: {

        // } break;

        
        case RS_PARSE_REQUEST_CURR_CELL_CSQ: {
            UART_WriteData((uint8_t *)at_cmds[AT_GET_CSQ], strlen(at_cmds[AT_GET_CSQ]));

            parse_state = PS_PARSE_AWAIT_CURR_CELL_CSQ;
        } break;
        
        case PS_PARSE_AWAIT_CURR_CELL_CSQ: {
            if (strstr(rx_ring, "OK") && strstr(rx_ring, "+CSQ")) {
                ESP_LOGD(DBG_TAG, "%s", rx_ring);

                if (strlen(rx_ring) < RX_WORK_BUF_SIZE) {
                    memcpy(rx_work_buf, rx_ring, strlen(rx_ring) * sizeof(char));
                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;
                }

                parse_state = RS_PARSE_POST_CURR_CELL_CSQ;
            }
        } break;
        
        case RS_PARSE_POST_CURR_CELL_CSQ: {
            char *csq_str = find_char(rx_work_buf, ':', 1);
            csq_str++;  // Get to number

            // Find current carrier
            for (struct CarrierID_t *carriers = found_carriers; carriers < &found_carriers[MAX_NUM_CARRIERS]; carriers++) {
                if (carriers->stat == CELL_STAT_CURRENT) {
                    carriers->last_rssi = atoi(csq_str);
                    DISP_OnEvent(DISP_EV_CURR_CSQ, carriers->last_rssi);
                    ESP_LOGI(DBG_TAG, "Updated RSSI for (%d) to (%d)", carriers->numeric_oper, carriers->last_rssi);
                }
            }

            memset(rx_work_buf, 0, sizeof(rx_work_buf));
            parse_state = RS_PARSE_REQUEST_CURR_NMEA;
        } break;
        
        case RS_PARSE_REQUEST_CURR_NMEA: {
            UART_WriteData((uint8_t *)at_cmds[AT_REQUEST_NMEA_STRING], strlen(at_cmds[AT_REQUEST_NMEA_STRING]));

            parse_state = RS_PARSE_AWAIT_CURR_NMEA;
        } break;
        
        case RS_PARSE_AWAIT_CURR_NMEA: {
            if (strstr(rx_ring, "OK") && strstr(rx_ring, "+CGNSINF")) {
                ESP_LOGI(DBG_TAG, "%s", rx_ring);

                if (strlen(rx_ring) < RX_WORK_BUF_SIZE) {
                    memcpy(rx_work_buf, rx_ring, strlen(rx_ring) * sizeof(char));
                    memset(rx_ring, 0, sizeof(rx_ring));
                    rx_ring_w = rx_ring;
                }

                parse_state = RS_PARSE_POST_CURR_NMEA;
            }
        } break;

        case RS_PARSE_POST_CURR_NMEA: {
            ESP_LOGD(DBG_TAG, "NMEA RESPONSE: %s", rx_work_buf);

            char *search_ptr = NULL;
            
            // Get GNSS run status
            search_ptr = find_char(rx_work_buf, ':', 1);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS RUN STATUS");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.gnss_on = (bool)atoi(search_ptr);

            // Get fix status
            search_ptr = find_char(rx_work_buf, ',', 1);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS fix statuts");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.fix_acquired = (bool)atoi(search_ptr);

            // Get time
            search_ptr = find_char(rx_work_buf, ',', 2);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS time");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.utc_date_time = atoll(search_ptr);
            long long int gnss_time = curr_fix.utc_date_time;
            curr_fix.secs = gnss_time % 100;
            gnss_time /= 100;

            curr_fix.mins = gnss_time % 100;
            gnss_time /= 100;

            curr_fix.hrs = gnss_time % 100;
            gnss_time /= 100;

            curr_fix.day = gnss_time % 100;
            gnss_time /= 100;

            curr_fix.month = gnss_time % 100;
            gnss_time /= 100;

            curr_fix.year = gnss_time % 10000;

            // Get latitude
            search_ptr = find_char(rx_work_buf, ',', 3);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS latitude");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.lat = (int)(atof(search_ptr) * 1000000.0f);
            
            // Get longitude
            search_ptr = find_char(rx_work_buf, ',', 4);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS longitude");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.lon = (int)(atof(search_ptr) * 1000000.0f);

            // Get altitude
            search_ptr = find_char(rx_work_buf, ',', 5);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS altitude");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.alt = (int)(atof(search_ptr));

            // Get SOG (kmh)
            search_ptr = find_char(rx_work_buf, ',', 6);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS SOG");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.sog_kmh = (int)(atof(search_ptr) * 100.0f);
            curr_fix.sog_mph = (int)(atof(search_ptr) * 0.621371f);

            // Get COG
            search_ptr = find_char(rx_work_buf, ',', 7);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS COG");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.cog_deg = (int)(atof(search_ptr) * 100.0f);

            // Get GNSS sats in view
            search_ptr = find_char(rx_work_buf, ',', 14);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS SIV");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.sats_in_view = atoi(search_ptr);

            // Get GNSS sats in use
            search_ptr = find_char(rx_work_buf, ',', 15);
            if (!search_ptr) {
                ESP_LOGW(DBG_TAG, "Parse error on GNSS SIU");
                goto rs_parse_post_curr_nmea_cleanup;
            }
            curr_fix.sats_in_use = atoi(search_ptr);

            if (curr_fix.fix_acquired) {
                DISP_OnEvent(DISP_EV_SAT_FIX, true);
            } else {
                DISP_OnEvent(DISP_EV_SAT_FIX, false);
            }

            DISP_OnEvent(DISP_EV_SAT_IN_VIEW, curr_fix.sats_in_view);
            DISP_OnEvent(DISP_EV_SAT_IN_USE, curr_fix.sats_in_use);
            DISP_OnEvent(DISP_EV_SPEED_MPH, curr_fix.sog_mph);
            DISP_OnEvent(DISP_EV_COG_DEGREES, curr_fix.cog_deg);

            ESP_LOGI(DBG_TAG, "Update curr fix:\n" \
                              "\tOn: %d\n" \
                              "\tFix: %d\n" \
                              "\tUTC date time: %lld\n" \
                              "\tLat: %d\n" \
                              "\tLon: %d\n" \
                              "\tAlt: %d\n" \
                              "\tSOG (km/h): %d\n" \
                              "\tSOG (mph):  %d\n" \
                              "\tCOG (degs): %d\n" \
                              "\tYYYY: %d\n" \
                              "\tMNTH: %d\n" \
                              "\tDD:   %d\n" \
                              "\tHH:   %d\n" \
                              "\tMM:   %d\n" \
                              "\tSS:   %d\n" \
                              "\tSIV:  %d\n" \
                              "\tSIU:  %d\n",

                              curr_fix.gnss_on, curr_fix.fix_acquired, curr_fix.utc_date_time, curr_fix.lat,
                              curr_fix.lon, curr_fix.alt, curr_fix.sog_kmh, curr_fix.sog_mph, curr_fix.cog_deg, curr_fix.year,
                              curr_fix.month, curr_fix.day, curr_fix.hrs, curr_fix.mins, curr_fix.secs,
                              curr_fix.sats_in_view, curr_fix.sats_in_use
            );


rs_parse_post_curr_nmea_cleanup:
            memset(rx_work_buf, 0, sizeof(rx_work_buf));
            parse_state = RS_PARSE_REQUEST_CURR_CELL_CSQ;
        } break;

        default: {
            ESP_LOGE(DBG_TAG, "RS_PARSE IN BAD STATE!!!");
            parse_state = RS_PARSE_UNKNOWN;
        }
    }
}

char *find_char(char *str, char c, unsigned int instance) {
    char *str_tmp = str;

    for (unsigned int i = 1; i <= instance && str_tmp != NULL; i++) {
        str_tmp = strchr(str_tmp, c);
        str_tmp++;
    }

    return str_tmp;
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