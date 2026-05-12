/**
 * @file main/main.c
 * @brief MQ135 proof‑of‑concept – voltage, resistance, ratio, and CO₂ ppm.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mq135.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(mq135_init());

    /* Wait for sensor heater to stabilise (20 minutes in production,
       at least 5 minutes for a quick test). */
    vTaskDelay(pdMS_TO_TICKS(300000)); // 5 minutes

    /* Calibrate in clean air – hold 5 seconds for the reading to settle */
    ESP_LOGI(TAG, "Calibrating in clean air...");
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_ERROR_CHECK(mq135_calibrate());

    while (1) {
        uint32_t raw;
        float voltage, ratio, ppm;

        if (mq135_read_raw(&raw) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        mq135_raw_to_voltage(raw, &voltage);
        mq135_get_ratio(raw, &ratio);
        mq135_get_ppm_co2(raw, &ppm);

        ESP_LOGI(TAG, "raw=%" PRIu32 " Vout=%.3f V Rs/R0=%.3f CO2=%.1f ppm",
                 raw, voltage, (double)ratio, (double)ppm);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}