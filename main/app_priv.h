/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SWITCH_POWER true
/* GPIO cho cảm biến */
#define PH_SENSOR_GPIO ADC1_CHANNEL_6
#define TURBIDITY_SENSOR_GPIO ADC1_CHANNEL_7
#define WATER_LEVEL_GPIO ADC1_CHANNEL_5 // Cảm biến mực nước (Digital)

#define PUMP_GPIO 26                          // Thêm GPIO cho bơm
#define DRAIN_GPIO CONFIG_EXAMPLE_OUTPUT_GPIO // Máy xả nước
#define SERVO_GPIO 18                         // GPIO cho servo

/* Chu kỳ cập nhật cảm biến (giây) */
#define SENSOR_UPDATE_PERIOD 30000

#define DEFAULT_TEMPERATURE 25.0
#define DEFAULT_PH 7.0
#define DEFAULT_NTU 0.0
#define REPORTING_PERIOD 60 /* Seconds */

#define ServoMsMin 0.06
#define ServoMsMax 2.1
#define ServoMsAvg ((ServoMsMax - ServoMsMin) / 2.0)

// Định nghĩa cấu hình I2C
#define I2C_MASTER_SCL_IO 22      /*!< GPIO cho I2C clock */
#define I2C_MASTER_SDA_IO 21      /*!< GPIO cho I2C data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number */
#define I2C_MASTER_FREQ_HZ 100000 /*!< Tần số clock I2C */

extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *switch_device;

extern esp_rmaker_device_t *ph_sensor_device;
extern esp_rmaker_device_t *turbidity_sensor_device;
extern esp_rmaker_device_t *pump_device;  // Thêm bơm vào
extern esp_rmaker_device_t *drain_device; // Máy xả nước
extern esp_rmaker_device_t *servo_device;

#define SERVO_MIN_PULSEWIDTH 1120 // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 1920 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90

void app_driver_init(void);
int app_driver_set_state(bool state);
bool app_driver_get_state(void);
float app_get_current_temperature();
float app_get_current_ph();
float app_get_current_turbidity();
void control_gpio(int gpio, bool state);
void display_sensor_data();
void check_sensor_and_control();