#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_err.h"

#include "app_network.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_types.h"
#include "esp_rmaker_standard_params.h"

static const char *TAG = "ALARM_NODE";

// ===================== GPIO PINS =====================
#define LED_GPIO     GPIO_NUM_2
#define BUZZER_GPIO  GPIO_NUM_7   // use the pin that worked in your buzzer test

// ===================== BUZZER PWM SETTINGS =====================
#define BUZZER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER      LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define BUZZER_FREQUENCY_HZ    2000
#define BUZZER_DUTY            512   // ~50%

static bool alarm_on = false;
static esp_rmaker_param_t *power_param = NULL;

// ===================== LED INIT =====================
static void gpio_init_led(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_GPIO, 0);
}

// ===================== BUZZER PWM INIT =====================
static void buzzer_pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .duty_resolution  = BUZZER_LEDC_DUTY_RES,
        .timer_num        = BUZZER_LEDC_TIMER,
        .freq_hz          = BUZZER_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BUZZER_GPIO,
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void buzzer_on(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

static void buzzer_off(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

static void set_alarm_outputs(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);

    if (on) {
        buzzer_on();
    } else {
        buzzer_off();
    }

    ESP_LOGI(TAG, "Outputs set: %s", on ? "ON" : "OFF");
}

// ===================== RAINMAKER WRITE CALLBACK =====================
static esp_err_t alarm_write_cb(const esp_rmaker_device_t *device,
                                const esp_rmaker_param_t *param,
                                const esp_rmaker_param_val_t val,
                                void *priv_data,
                                esp_rmaker_write_ctx_t *ctx)
{
    const char *param_name = esp_rmaker_param_get_name(param);
    ESP_LOGI(TAG, "Write callback fired for param: %s", param_name);

    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        alarm_on = val.val.b;
        set_alarm_outputs(alarm_on);
        esp_rmaker_param_update_and_report((esp_rmaker_param_t *)param, val);

        ESP_LOGI(TAG, "Alarm changed from app: %s", alarm_on ? "ON" : "OFF");
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

    gpio_init_led();
    buzzer_pwm_init();

    // ===================== STARTUP SELF-TEST =====================
    ESP_LOGI(TAG, "Startup self-test: LED + buzzer ON for 1 second");
    set_alarm_outputs(true);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_alarm_outputs(false);
    ESP_LOGI(TAG, "Startup self-test complete");

    app_network_init();

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg,
                                                   "Smart Security Alarm",
                                                   "Smart Security");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise RainMaker node");
        abort();
    }

    esp_rmaker_device_t *alarm_device = esp_rmaker_device_create(
        "Alarm Unit",
        ESP_RMAKER_DEVICE_SWITCH,
        NULL
    );
    if (!alarm_device) {
        ESP_LOGE(TAG, "Could not create RainMaker device");
        abort();
    }

    esp_rmaker_device_add_cb(alarm_device, alarm_write_cb, NULL);

    esp_rmaker_device_add_param(
        alarm_device,
        esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Alarm Unit")
    );

    power_param = esp_rmaker_power_param_create(
        ESP_RMAKER_DEF_POWER_NAME,
        false
    );
    if (!power_param) {
        ESP_LOGE(TAG, "Could not create power param");
        abort();
    }

    esp_rmaker_device_add_param(alarm_device, power_param);
    esp_rmaker_device_assign_primary_param(alarm_device, power_param);

    esp_rmaker_node_add_device(node, alarm_device);

    esp_rmaker_start();

    err = app_network_start(POP_TYPE_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start network provisioning");
        abort();
    }

    while (1) {
        // keep hardware synced to software state
        set_alarm_outputs(alarm_on);

        ESP_LOGI(TAG, "Alive | alarm_on=%s", alarm_on ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}