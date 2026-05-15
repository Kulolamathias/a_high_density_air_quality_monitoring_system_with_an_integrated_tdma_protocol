/**
 * @file main/tx_main.c
 * @brief Transmitter node for the TDMA air‑quality monitoring system.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This application runs on each ESP32 that acts as a sensor transmitter.
 * It uses the common network module, the MQ135 sensor driver, and implements
 * a TDMA transmission protocol. It publishes air‑quality data on a shared
 * MQTT topic only during its assigned time slot.
 *
 * =============================================================================
 * TDMA PROTOCOL
 * =============================================================================
 * 1. Node ID = "TX_" + last 4 hex digits of Wi‑Fi MAC.
 * 2. After network start, the transmitter attempts to join a slot by sending
 *    {"cmd":"join","id":"TX_XXXX"} to airquality/request.
 * 3. It subscribes to airquality/control and waits for an assignment message:
 *    {"cmd":"assign","id":"TX_XXXX","slot":0,"offset_ms":0,"duration_ms":5000,"period_ms":10000}
 * 4. If no assignment arrives within 5 seconds, the transmitter falls back to
 *    a static slot: slot = (MAC.last_byte % 2), with the same period/duration.
 * 5. During its slot (every period_ms), it publishes sensor data to
 *    airquality/data exactly once.
 *
 * =============================================================================
 * OWNERSHIP
 * =============================================================================
 * - Owns: slot parameters, static sensor handle.
 * - Provides: slot‑aware publication loop.
 * - Does NOT: manage slots, display data, or process other nodes' data.
 *
 * @author Matthithyahu
 * @date 2026/05/12
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_mac.h"
#include <time.h> 
#include "cJSON.h"

#include "network.h"   /**< Step 1 network module */
#include "mq135.h"     /**< MQ135 sensor driver (from earlier) */

static const char *TAG = "TX";

/* -------------------------------------------------------------------------
 * TDMA slot defaults (used if no dynamic assignment)
 * ------------------------------------------------------------------------- */
#define TDMA_SLOT_PERIOD_MS     10000   /**< Super‑frame length */
#define TDMA_SLOT_DURATION_MS    5000   /**< How long each slot can transmit */
#define TDMA_JOIN_TIMEOUT_MS     5000   /**< Time to wait for slot assignment */

/* -------------------------------------------------------------------------
 * Static slot fallback – determined by MAC
 * ------------------------------------------------------------------------- */
static uint8_t  s_slot_number     = 0;   /**< 0 or 1 */
static uint32_t s_slot_offset_ms  = 0;   /**< Start of our window */
static uint32_t s_slot_duration_ms = TDMA_SLOT_DURATION_MS;
static uint32_t s_slot_period_ms  = TDMA_SLOT_PERIOD_MS;

static char s_node_id[16];           /**< "TX_XXXX" */

/* Flag to know whether we received a dynamic assignment */
static bool s_assigned = false;

/* -------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */
static void request_slot(void);
static void handle_control_message(const char *topic,
                                   const char *data,
                                   size_t data_len);
static void read_sensor_and_publish(void);

/* ========================================================================= */
void app_main(void)
{
    esp_err_t ret;

    /* ---- 1. Start network (Wi‑Fi, MQTT, NTP) ---- */
    ret = network_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "network_start failed: %s", esp_err_to_name(ret));
        return;
    }

    /* ---- 2. Get our node ID ---- */
    network_get_node_id(s_node_id, sizeof(s_node_id), "TX");
    ESP_LOGI(TAG, "Node ID: %s", s_node_id);

    /* ---- 3. Initialise MQ135 sensor ---- */
    ret = mq135_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mq135_init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Give the sensor some time to warm up (heater) */
    ESP_LOGI(TAG, "Waiting 30 seconds for MQ135 heater...");
    vTaskDelay(pdMS_TO_TICKS(30000));

    /* Calibrate R0 in clean air */
    ESP_LOGI(TAG, "Calibrating MQ135 (ensure clean air)");
    ret = mq135_calibrate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration failed");
    }

    /* ---- 4. Register for incoming control messages ---- */
    network_mqtt_set_data_callback(handle_control_message);
    network_mqtt_subscribe("airquality/control", 1);

    /* ---- 5. Request a TDMA slot (dynamic assignment) ---- */
    request_slot();

    /* ---- 6. If no dynamic assignment, fall back to static slot ---- */
    if (!s_assigned) {
        /* Derive slot from last byte of MAC */
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        s_slot_number = mac[5] % 2;          /**< 0 or 1 */
        s_slot_offset_ms = s_slot_number * s_slot_duration_ms;
        s_slot_period_ms = TDMA_SLOT_PERIOD_MS;
        s_slot_duration_ms = TDMA_SLOT_DURATION_MS;
        ESP_LOGI(TAG, "No dynamic assignment – using static slot %d, offset %lu ms",
                 s_slot_number, (unsigned long)s_slot_offset_ms);
    }

    /* ---- 7. Main publication loop ---- */
    while (1) {
        /* Calculate the next slot start (absolute time in us) */
        int64_t now_us = esp_timer_get_time();   /**< microseconds since boot */
        int64_t now_ms = now_us / 1000;

        /* Determine how many super‑frames have passed */
        uint64_t superframe_start = (now_ms / s_slot_period_ms) * s_slot_period_ms;
        int64_t slot_start_ms = superframe_start + s_slot_offset_ms;
        int64_t slot_end_ms = slot_start_ms + s_slot_duration_ms;

        int64_t wait_ms;

        if (now_ms >= slot_start_ms && now_ms < slot_end_ms) {
            /* We are inside our slot – publish immediately */
            read_sensor_and_publish();
            /* Then wait until the slot ends + a guard interval */
            wait_ms = slot_end_ms - now_ms + 100;   /**< 100 ms guard */
        } else if (now_ms < slot_start_ms) {
            /* Slot hasn't started yet – sleep until it does */
            wait_ms = slot_start_ms - now_ms;
        } else {
            /* Slot is in the future (next superframe) */
            slot_start_ms += s_slot_period_ms;
            wait_ms = slot_start_ms - now_ms;
        }

        ESP_LOGD(TAG, "Sleeping for %lld ms until next slot", (long long)wait_ms);
        vTaskDelay(pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1));
    }
}

/* ========================================================================= */
static void request_slot(void)
{
    /* Build JSON join request */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "cmd", "join");
    cJSON_AddStringToObject(root, "id", s_node_id);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json) {
        network_mqtt_publish("airquality/request", json, strlen(json), 1, false);
        ESP_LOGI(TAG, "Sent join request: %s", json);
        free(json);
    }

    /* Wait for assignment (the callback will set s_assigned) */
    int64_t start = esp_timer_get_time() / 1000;
    while (!s_assigned) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if ((esp_timer_get_time() / 1000 - start) > TDMA_JOIN_TIMEOUT_MS) {
            ESP_LOGW(TAG, "No slot assignment received within timeout");
            break;
        }
    }
}

/* ========================================================================= */
static void handle_control_message(const char *topic,
                                   const char *data,
                                   size_t data_len)
{
    /* Only process airquality/control */
    if (strcmp(topic, "airquality/control") != 0) return;

    /* Parse JSON */
    cJSON *root = cJSON_ParseWithLength(data, data_len);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse control message");
        return;
    }

    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "assign") == 0) {
        cJSON *id = cJSON_GetObjectItem(root, "id");
        if (id && cJSON_IsString(id) && strcmp(id->valuestring, s_node_id) == 0) {
            /* This assignment is for us */
            cJSON *slot     = cJSON_GetObjectItem(root, "slot");
            cJSON *offset   = cJSON_GetObjectItem(root, "offset_ms");
            cJSON *duration = cJSON_GetObjectItem(root, "duration_ms");
            cJSON *period   = cJSON_GetObjectItem(root, "period_ms");

            if (slot && offset && duration && period) {
                s_slot_number     = (uint8_t)slot->valueint;
                s_slot_offset_ms  = (uint32_t)offset->valueint;
                s_slot_duration_ms = (uint32_t)duration->valueint;
                s_slot_period_ms  = (uint32_t)period->valueint;
                s_assigned = true;
                ESP_LOGI(TAG, "Slot assigned: slot=%d, offset=%lu ms, dur=%lu ms, per=%lu ms",
                         s_slot_number,
                         (unsigned long)s_slot_offset_ms,
                         (unsigned long)s_slot_duration_ms,
                         (unsigned long)s_slot_period_ms);
            }
        }
    }

    cJSON_Delete(root);
}

/* ========================================================================= */
static void read_sensor_and_publish(void)
{
    uint32_t raw;
    float volt, ratio, ppm;

    if (mq135_read_raw(&raw) != ESP_OK) {
        ESP_LOGW(TAG, "Sensor read failed, skipping publication");
        return;
    }

    mq135_raw_to_voltage(raw, &volt);
    mq135_get_ratio(raw, &ratio);
    mq135_get_ppm_co2(raw, &ppm);

    /* Build JSON payload */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "id", s_node_id);
    cJSON_AddNumberToObject(root, "raw", raw);
    cJSON_AddNumberToObject(root, "V", volt);
    cJSON_AddNumberToObject(root, "ratio", ratio);
    cJSON_AddNumberToObject(root, "ppm", ppm);
    cJSON_AddNumberToObject(root, "ts", (double)time(NULL));

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json) {
        network_mqtt_publish("airquality/data", json, strlen(json), 1, false);
        ESP_LOGI(TAG, "Published: %s", json);
        free(json);
    }
}