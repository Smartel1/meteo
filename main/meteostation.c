#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include "qmc5883l.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define HALL_GPIO GPIO_NUM_12
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22

static uint8_t gpio12state = 0;
static int64_t prevTickMs = 0;

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


void app_main(void) {
    init_led();
    init_hall_sensor();
    init_compass();

    while (0) {
        int hall_sensor_on = gpio_get_level(12) == 0;
        if (gpio12state != hall_sensor_on) {
            gpio_set_level(BLINK_GPIO, hall_sensor_on);
            gpio12state = hall_sensor_on;
            if (gpio12state == 0) {
                int64_t time = esp_timer_get_time();
                if (prevTickMs != 0) {
                    float freq = 1 / ((float)(time - prevTickMs) / 1000000);
                    ESP_LOGI(TAG, "frequency is %f hz", freq);
                }
                prevTickMs = time;
            }
            ESP_LOGI(TAG, "Hall sensor is %s!", hall_sensor_on ? "ON" : "OFF");
        }
        vTaskDelay(5);
    }

    int16_t azimuth;
    int16_t temp;
    while (1) {
        qmc5883l_get_azimuth(&compass_settings, &azimuth);
        qmc5883l_get_temp(&compass_settings, &temp);
        ESP_LOGI(TAG, "azimuth = %d temp = %d", azimuth, temp);
        vTaskDelay(50);
    }
}