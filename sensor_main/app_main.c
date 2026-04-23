#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_err.h"

#include "app_network.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"

static const char *TAG = "SENSOR_NODE";

// ===================== GPIO PINS =====================
#define PIR_GPIO   GPIO_NUM_5
#define REED_GPIO  GPIO_NUM_4

// ===================== GLOBAL STATE =====================
static bool armed = true;

// RainMaker parameter handles
static esp_rmaker_param_t *armed_param = NULL;
static esp_rmaker_param_t *motion_param = NULL;
static esp_rmaker_param_t *door_param = NULL;
static esp_rmaker_param_t *alarm_param = NULL;

// ===================== GPIO INIT =====================
static void gpio_init_inputs(void)
{
    gpio_config_t pir_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&pir_conf));

    gpio_config_t reed_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << REED_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&reed_conf));
}

// ===================== RAINMAKER WRITE CALLBACK =====================
static esp_err_t sensor_write_cb(const esp_rmaker_device_t *device,
                                 const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val,
                                 void *priv_data,
                                 esp_rmaker_write_ctx_t *ctx)
{
    const char *param_name = esp_rmaker_param_get_name(param);

    ESP_LOGI(TAG, "Write callback fired for param: %s", param_name);

    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        armed = val.val.b;
        esp_rmaker_param_update_and_report((esp_rmaker_param_t *)param, val);
        ESP_LOGI(TAG, "Armed changed from app: %s", armed ? "ON" : "OFF");
    }

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_init_inputs();

    app_network_init();

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg,
                                                   "Security Sensor",
                                                   "Smart Security");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise RainMaker node");
        abort();
    }

    esp_rmaker_device_t *device = esp_rmaker_device_create(
        "Sensor Unit",
        ESP_RMAKER_DEVICE_SWITCH,
        NULL
    );
    if (!device) {
        ESP_LOGE(TAG, "Could not create RainMaker device");
        abort();
    }

    esp_rmaker_device_add_cb(device, sensor_write_cb, NULL);

    esp_rmaker_device_add_param(
        device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Sensor Unit")
    );

    armed_param = esp_rmaker_power_param_create(
        ESP_RMAKER_DEF_POWER_NAME,
        true
    );

    motion_param = esp_rmaker_param_create(
        "Motion",
        "esp.param.motion",
        esp_rmaker_bool(false),
        PROP_FLAG_READ
    );

    door_param = esp_rmaker_param_create(
        "Door",
        "esp.param.contact",
        esp_rmaker_bool(false),
        PROP_FLAG_READ
    );

    alarm_param = esp_rmaker_param_create(
        "Alarm",
        "esp.param.alert",
        esp_rmaker_bool(false),
        PROP_FLAG_READ
    );

    if (!armed_param || !motion_param || !door_param || !alarm_param) {
        ESP_LOGE(TAG, "Could not create one or more RainMaker params");
        abort();
    }

    esp_rmaker_device_add_param(device, armed_param);
    esp_rmaker_device_add_param(device, motion_param);
    esp_rmaker_device_add_param(device, door_param);
    esp_rmaker_device_add_param(device, alarm_param);

    esp_rmaker_device_assign_primary_param(device, armed_param);

    esp_rmaker_node_add_device(node, device);

    esp_rmaker_start();

    err = app_network_start(POP_TYPE_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start network provisioning");
        abort();
    }

    ESP_LOGI(TAG, "Warming up PIR for 60 seconds...");
    vTaskDelay(pdMS_TO_TICKS(60000));

    int pir_hold = 0;

    bool last_armed = armed;
    bool last_motion = false;
    bool last_door = false;
    bool last_alarm = false;
    bool first_report = true;

    while (1) {
        int pir_raw = gpio_get_level(PIR_GPIO);
        int reed_raw_original = gpio_get_level(REED_GPIO);

        // invert so magnet near = 1
        int reed_raw = !reed_raw_original;

        bool motion_detected = false;
        bool magnet_near = (reed_raw == 1);
        bool alarm_triggered = false;
        bool anything_changed = false;

        if (pir_raw == 1) {
            pir_hold = 3;   // 3 x 1000 ms = 3 seconds hold
        }

        if (pir_hold > 0) {
            motion_detected = true;
            pir_hold--;
        }

        if (armed && (motion_detected || magnet_near)) {
            alarm_triggered = true;
        }

        if (first_report || armed != last_armed) {
            esp_rmaker_param_update(armed_param, esp_rmaker_bool(armed));
            last_armed = armed;
            anything_changed = true;
        }

        if (first_report || motion_detected != last_motion) {
            esp_rmaker_param_update(motion_param, esp_rmaker_bool(motion_detected));
            last_motion = motion_detected;
            anything_changed = true;
        }

        if (first_report || magnet_near != last_door) {
            esp_rmaker_param_update(door_param, esp_rmaker_bool(magnet_near));
            last_door = magnet_near;
            anything_changed = true;
        }

        if (first_report || alarm_triggered != last_alarm) {
            esp_rmaker_param_update(alarm_param, esp_rmaker_bool(alarm_triggered));
            last_alarm = alarm_triggered;
            anything_changed = true;
        }

        if (anything_changed) {
            esp_rmaker_report_updated_params();
        }

        first_report = false;

        ESP_LOGI(TAG,
                 "PIR=%d | REED_ORIG=%d | REED_INV=%d | motion=%s | magnet_near=%s | alarm=%s | armed=%s",
                 pir_raw,
                 reed_raw_original,
                 reed_raw,
                 motion_detected ? "true" : "false",
                 magnet_near ? "true" : "false",
                 alarm_triggered ? "true" : "false",
                 armed ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}