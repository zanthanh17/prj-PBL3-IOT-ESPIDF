#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <driver/adc.h>
#include "driver/ledc.h"
#include <driver/i2c.h>
#include "ssd1306.h"
#include <esp_log.h>
#include <math.h>
#include "driver/mcpwm.h"

#include <iot_button.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

static TimerHandle_t sensor_timer;

/* Giá trị cảm biến hiện tại */
static float g_temperature = DEFAULT_TEMPERATURE;
static float g_ph_value = DEFAULT_PH;         // Giá trị pH ban đầu
static float g_turbidity_value = DEFAULT_NTU; // Giá trị độ đục ban đầu
static bool g_power_state = false;

// Biến toàn cục cho SSD1306
static ssd1306_handle_t ssd1306_dev = NULL;

static const char *TAG = "app_driver";

/* Hàm đọc giá trị từ cảm biến nhiệt độ */
static float read_temp_sensor()
{
    static float delta = 0.5;
    g_temperature += delta;
    if (g_temperature > 99)
    {
        delta = -0.5;
    }
    else if (g_temperature < 1)
    {
        delta = 0.5;
    }

    // Ghi log giá trị
    return DEFAULT_TEMPERATURE;
}
/* Hàm đọc giá trị từ cảm biến pH */
static float read_ph_sensor()
{
    int raw = adc1_get_raw(PH_SENSOR_GPIO);
    float voltage = raw * (3.3 / 4095); // Chuyển đổi giá trị ADC thành điện áp (0-3.3V)

    // Tính giá trị pH dựa trên công thức
    float ph = 20.5940 - (5.4450 * voltage);

    // Ghi log giá trị
    return ph;
}

/* Hàm đọc giá trị từ cảm biến độ đục */
static float read_turbidity_sensor()
{
    // Đọc giá trị ADC
    int raw = adc1_get_raw(TURBIDITY_SENSOR_GPIO);
    float voltage = raw * (3.3 / 4095); // Chuyển đổi giá trị ADC thành điện áp (3.3V tham chiếu)

    // Lấy trung bình 800 lần đọc để giảm nhiễu
    float sum_voltage = 0;
    for (int i = 0; i < 800; i++)
    {
        raw = adc1_get_raw(TURBIDITY_SENSOR_GPIO);
        sum_voltage += raw * (3.3 / 4095);
    }
    voltage = sum_voltage / 800;

    // Tính giá trị NTU
    float NTU = 0;
    if (voltage < 0.36)
    {
        NTU = 3000; // Giới hạn giá trị NTU tối đa
    }
    else if (voltage > 1.8)
    {
        NTU = 0; // Giá trị NTU tối thiểu
    }
    else
    {
        NTU = (-1120.4 * (voltage + 2.4) * (voltage + 2.4)) + (5742.3 * (voltage + 2.4)) - 4352.9;
    };
    return NTU;
}

/* Hàm đọc cảm biến mực nước */
static bool is_water_level_high()
{
    int water_level = gpio_get_level(WATER_LEVEL_GPIO);
    ESP_LOGI(TAG, "Water level sensor state: %d", water_level);
    return water_level; // Trả về true nếu phát hiện nước
}

void control_gpio(int gpio, bool state)
{
    gpio_set_level(gpio, state ? 1 : 0);
    ESP_LOGI(TAG, "GPIO %d set to %s", gpio, state ? "ON" : "OFF");
}

void check_sensor_and_control()
{
    // Kiểm tra điều kiện tự động bật máy bơm
    bool pump_state = (g_turbidity_value > 500);
    control_gpio(PUMP_GPIO, pump_state);

    // Kiểm tra điều kiện tự động bật van xả
    bool drain_state = (is_water_level_high() == 1);
    pump_state = false;
    control_gpio(DRAIN_GPIO, drain_state);
}

void servoDeg0()
{

    int duty = (int)(100.0 * (ServoMsMin / 20.0) * 81.91);
    printf("%fms, duty = %f%% -> %d\n", ServoMsMin, 100.0 * (ServoMsMin / 20.0), duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void servoDeg90()
{

    int duty = (int)(100.0 * (ServoMsAvg / 20.0) * 81.91);
    printf("%fms, duty = %f%% -> %d\n", ServoMsAvg, 100.0 * (ServoMsAvg / 20.0), duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void servoDeg180()
{
    int duty = (int)(100.0 * (ServoMsMax / 20.0) * 81.91);
    printf("%fms, duty = %f%% -> %d\n", ServoMsMax, 100.0 * (ServoMsMax / 20.0), duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

/*Hàm điều khiển góc quay*/
static void app_indicator_set(bool state)
{
    if (state)
    {

        servoDeg180();
    }
    else
    {
        servoDeg90();
    }
}

/*Hàm cấu hình cho servo*/
static void app_indicator_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_GPIO,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
    app_indicator_set(g_power_state);
}

static void set_power_state(bool target)
{
    gpio_set_level(SERVO_GPIO, target);
    app_indicator_set(target);
}

static void app_sensor_update(TimerHandle_t handle)
{

    g_ph_value = read_ph_sensor();
    g_turbidity_value = read_turbidity_sensor();

    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
        esp_rmaker_float(g_temperature));

    // Cập nhật pH nếu thay đổi

    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_name(ph_sensor_device, "pH"),
        esp_rmaker_float(g_ph_value));

    // Cập nhật độ đục nếu thay đổi
    esp_rmaker_param_update_and_report(
        esp_rmaker_device_get_param_by_name(turbidity_sensor_device, "ntu"),
        esp_rmaker_float(g_turbidity_value));

    ESP_LOGI(TAG, "pH: %.2f, Turbidity: %.2f NTU, Temp: %.2f C", g_ph_value, g_turbidity_value, g_temperature);
}

// Hàm khởi tạo I2C
void i2c_master_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Hàm hiển thị dữ liệu cảm biến lên OLED
void display_sensor_data()
{
    char buffer[32];

    // Xóa màn hình
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    // Hiển thị nhiệt độ
    sprintf(buffer, "Temp: %.2f C", g_temperature);
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)buffer, 16, 1);

    // Hiển thị pH
    sprintf(buffer, "pH: %.2f", g_ph_value);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)buffer, 16, 1);

    // Hiển thị độ đục
    sprintf(buffer, "Turb: %.2f NTU", g_turbidity_value);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)buffer, 16, 1);

    // Cập nhật dữ liệu hiển thị
    ssd1306_refresh_gram(ssd1306_dev);
}

float app_get_current_temperature()
{
    return DEFAULT_TEMPERATURE;
}

float app_get_current_ph()
{
    return g_ph_value;
}

float app_get_current_turbidity()
{
    return g_turbidity_value;
}

esp_err_t app_sensor_init(void)
{

    // Cấu hình ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PH_SENSOR_GPIO, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TURBIDITY_SENSOR_GPIO, ADC_ATTEN_DB_11);

    // Cấu hình GPIO cho cảm biến mực nước
    gpio_config_t io_conf = {
        .pin_bit_mask = ((uint64_t)1 << WATER_LEVEL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Đặt mặc định trạng thái máy xả nước
    gpio_set_level(DRAIN_GPIO, 0);
    gpio_set_level(PUMP_GPIO, 0);
    gpio_set_level(WATER_LEVEL_GPIO, 0);

    g_temperature = DEFAULT_TEMPERATURE;
    g_ph_value = DEFAULT_PH; // Giá trị pH ban đầu
    g_turbidity_value = DEFAULT_NTU;

    sensor_timer = xTimerCreate("app_sensor_update_tm", SENSOR_UPDATE_PERIOD / portTICK_PERIOD_MS,
                                pdTRUE, NULL, app_sensor_update);
    if (sensor_timer)
    {
        xTimerStart(sensor_timer, 0);
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_driver_init()
{
    // Khởi tạo I2C
    i2c_master_init();

    // Khởi tạo OLED SSD1306
    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    /* Configure power */
    gpio_config_t servo_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Không cần kéo lên
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Không cần kéo xuống
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = ((uint64_t)1 << SERVO_GPIO) | ((uint64_t)1 << PUMP_GPIO) | ((uint64_t)1 << DRAIN_GPIO)};
    gpio_config(&servo_conf);
    app_indicator_init();
    app_sensor_init();
}

int IRAM_ATTR app_driver_set_state(bool state)
{

    set_power_state(state);

    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}
