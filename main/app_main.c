#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_standard_types.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

static const char *TAG = "app_main";

/* Thêm vào phần khai báo */
esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *ph_sensor_device;
esp_rmaker_device_t *turbidity_sensor_device;
esp_rmaker_device_t *pump_device;  // Thêm bơm vào
esp_rmaker_device_t *drain_device; // Máy xả nước
esp_rmaker_device_t *servo_device;

/* Hàm callback trạng thái của servo */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                          const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx)
    {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    const char *device_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0)
    {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                 val.val.b ? "true" : "false", device_name, param_name);
        if (strcmp(device_name, "Servo") == 0)
        {
            app_driver_set_state(val.val.b);
        }
    }
    else
    {
        /* Silently ignoring invalid params */
        return ESP_OK;
    }
    esp_rmaker_param_update(param, val);
    return ESP_OK;
}

/* Hàm callback trạng thái của máy bơm */
// static esp_err_t pump_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
//                                const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
// {
//     if (ctx)
//     {
//         ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
//     }
//     const char *device_name = esp_rmaker_device_get_name(device);
//     const char *param_name = esp_rmaker_param_get_name(param);
//     if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0)
//     {
//         ESP_LOGI(TAG, "Received value = %s for %s - %s",
//                  val.val.b ? "true" : "false", device_name, param_name);
//         if (strcmp(device_name, "Water Pump") == 0)
//         {
//             control_gpio(PUMP_GPIO, val.val.b);
//         }
//     }
//     else
//     {
//         /* Silently ignoring invalid params */
//         return ESP_OK;
//     }
//     esp_rmaker_param_update(param, val);
//     return ESP_OK;
// }

/* Hàm callback trạng thái của máy xả */
static esp_err_t drain_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx)
    {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    const char *device_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0)
    {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                 val.val.b ? "true" : "false", device_name, param_name);
        if (strcmp(device_name, "Water Drain") == 0)
        {
            control_gpio(DRAIN_GPIO, val.val.b);
        }
    }
    else
    {
        /* Silently ignoring invalid params */
        return ESP_OK;
    }
    esp_rmaker_param_update(param, val);
    return ESP_OK;
}

void app_main()
{

    app_driver_init();

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Multi Device", "Multi Device");
    if (!node)
    {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }
    /* Create a Switch device and add the relevant parameters to it */
    servo_device = esp_rmaker_switch_device_create("Servo", NULL, DEFAULT_SWITCH_POWER);
    esp_rmaker_device_add_cb(servo_device, write_cb, NULL);
    esp_rmaker_node_add_device(node, servo_device);

    /* Create a Temperature Sensor device and add the relevant parameters to it */
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Temperature Sensor", NULL, app_get_current_temperature());
    esp_rmaker_node_add_device(node, temp_sensor_device);

    // Thiết bị cảm biến pH
    ph_sensor_device = esp_rmaker_ph_sensor_device_create("pH Sensor", NULL, app_get_current_ph());
    esp_rmaker_node_add_device(node, ph_sensor_device);

    // Thiết bị cảm biến độ đục
    turbidity_sensor_device = esp_rmaker_turbidity_sensor_device_create("Turbidity Sensor", NULL, app_get_current_turbidity());
    esp_rmaker_node_add_device(node, turbidity_sensor_device);

    // Khởi tạo thiết bị bơm
    // pump_device = esp_rmaker_switch_device_create("Water Pump", NULL, false);
    // esp_rmaker_device_add_cb(pump_device, pump_write_cb, NULL);
    // esp_rmaker_node_add_device(node, pump_device);

    // Khởi tạo thiết bị máy xả nước
    drain_device = esp_rmaker_switch_device_create("Water Drain", NULL, false);
    esp_rmaker_device_add_cb(drain_device, drain_write_cb, NULL);
    esp_rmaker_node_add_device(node, drain_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    esp_rmaker_start();

    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }

    app_driver_set_state(DEFAULT_SWITCH_POWER);
    // xTaskCreate(DFPlay, "DFPlay", 1024 * 4, NULL, 2, NULL);
    while (1)
    {
        app_get_current_temperature();
        app_get_current_ph();
        app_get_current_turbidity();
        display_sensor_data();
        check_sensor_and_control();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
