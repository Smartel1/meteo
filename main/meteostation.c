#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include <string.h>
#include <driver/adc.h>
#include "qmc5883l.h"
#include "sim800l.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define HALL_GPIO GPIO_NUM_12
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22

// калибровки компаса
static uint8_t X_OFFSET = 150;
static float X_SCALE = 3.2f;
static uint8_t Y_OFFSET = 185;
static float Y_SCALE = 3.03f;

static qmc5883l_settings compass_settings;

static void init_led(void) {
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void init_hall_sensor(void) {
    gpio_reset_pin(HALL_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(HALL_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(HALL_GPIO);
}

static float get_wind_speed(void) {
    ESP_LOGI(TAG, "Measuring wind speed");
    uint8_t rounds_count = 0;
    bool prev_tick_hall_sensor_on = 0;
    int64_t start_measurement_time = esp_timer_get_time();
    while (esp_timer_get_time() - start_measurement_time < 5000000) {
        bool current_tick_hall_sensor_on = gpio_get_level(12) == 0;
        if (prev_tick_hall_sensor_on != current_tick_hall_sensor_on) {
            gpio_set_level(BLINK_GPIO, current_tick_hall_sensor_on);
            prev_tick_hall_sensor_on = current_tick_hall_sensor_on;
            if (prev_tick_hall_sensor_on) {
                rounds_count++;
            }
        }
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "Rounds made in 5 sec: %d", rounds_count);
    return rounds_count;
}

static uint8_t get_azimuth(void) {
    int16_t azimuth;
    qmc5883l_get_azimuth(&compass_settings, &azimuth);
    ESP_LOGI(TAG, "azimuth = %d", azimuth);
    return azimuth;
}

static uint8_t get_temp(void) {
    int16_t temp;
    qmc5883l_get_temp(&compass_settings, &temp);
    ESP_LOGI(TAG, "temp = %d", temp);
    return temp;
}

static void init_compass(void) {
    int i2c_master_port = 0;
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_SDA,         // select SDA GPIO specific to your project
            .scl_io_num = GPIO_SCL,         // select SCL GPIO specific to your project
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");

    qmc5883l_settings compass_settings_obj = {
            .port = i2c_master_port,
            .x_offset = X_OFFSET,
            .x_scale = X_SCALE,
            .y_offset = Y_OFFSET,
            .y_scale = Y_SCALE
    };
    compass_settings = compass_settings_obj;
    qmc5883l_init(&compass_settings);
    ESP_LOGI(TAG, "Compass set successfully");
}

void send_metrics(float wind_speed, int azimuth, int temperature, float voltage) {
    sendCommand("ATZ");
    sendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
    sendCommand("AT+SAPBR=3,1,\"APN\",\"internet.mts.ru\"");
    sendCommand("AT+SAPBR=1,1");
    sendCommand("AT+SAPBR=2,1");
    sendCommand("AT+HTTPINIT");
    sendCommand("AT+HTTPPARA=\"CID\",1");
    sendCommand("AT+HTTPPARA=\"URL\",\"http://193.124.125.33/metrics\"");
    sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    char request_body[100];
    sprintf(request_body, "{\"s\":%.1f,\"a\":%d,\"t\":%d,\"v\":%.1f}", wind_speed, azimuth, temperature, voltage);
    char param[25];
    sprintf(param, "AT+HTTPDATA=%d,20000", strlen(request_body));
    sendCommand(param);
    ESP_LOGI(TAG, "%s", param);
    ESP_LOGI(TAG, "%s", request_body);
    sendCommand(request_body);
    sendCommand("AT+HTTPACTION=1");
}

void app_main(void) {
    init_led();
    init_hall_sensor();
    init_compass();
    configureUART();
    turnOnSim800l();

    float wind_speed = get_wind_speed();
    uint8_t azimuth = get_azimuth();
    uint8_t temp = get_temp();
    float voltage = 3.2 * adc1_get_raw(ADC1_CHANNEL_0) / 4069;

    send_metrics(5.5f, 240, 27, 3.2f);
    turnOffSim800l();
}