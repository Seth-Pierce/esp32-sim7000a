#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "gpio_internal.h"

#define PWR_KEY GPIO_NUM_2

#define GPIO_LOW 0
#define GPIO_HIGH 1

static const char *DBG_TAG = "gpio";

void GPIO_StartDevice(void) {
    gpio_config_t io_cfg = {};

    io_cfg.intr_type = GPIO_INTR_DISABLE;
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = 1ULL<<GPIO_NUM_2;
    io_cfg.pull_down_en = 0;
    io_cfg.pull_up_en = 0;

    ESP_LOGI(DBG_TAG, "Configuring IO");

    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to configure IO: %u", err);
        return;
    }

    GPIO_StartCell();
}

void GPIO_StartCell(void) {
    esp_err_t err = ESP_OK;

    ESP_LOGV(DBG_TAG, "Toggling PWR_KEY HIGH");
    err = gpio_set_level(PWR_KEY, GPIO_HIGH);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to set level for %d to %d", PWR_KEY, GPIO_HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGV(DBG_TAG, "Toggling PWR_KEY LOW");
    err = gpio_set_level(PWR_KEY, GPIO_LOW);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to set level for %d to %d", PWR_KEY, GPIO_LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGV(DBG_TAG, "Toggling PWR_KEY HIGH");
    err = gpio_set_level(PWR_KEY, GPIO_HIGH);
    if (err != ESP_OK) {
        ESP_LOGE(DBG_TAG, "Failed to set level for %d to %d", PWR_KEY, GPIO_HIGH);
    }
}