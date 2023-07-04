#include <stdio.h>
#include <esp_timer.h>
#include <driver/i2c.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;
static uint8_t gpio12state = 0;
static int64_t prevTickMs = 0;
static uint8_t COMPASS_I2C_ADDRESS = 0x0D;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

//    while (1) {
        s_led_state = 1;
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
//    }
//    while (1) {
//        int signal = gpio_get_level(12);
//        if (gpio12state != signal) {
//            gpio12state = signal;
//            if (gpio12state == 0) {
//                int64_t time = esp_timer_get_time();
//                if (prevTickMs != 0) {
//                    float freq = 1 / ((float)(time - prevTickMs) / 1000000);
//                    ESP_LOGI(TAG, "frequency is %f hz", freq);
//                }
//                prevTickMs = time;
//            }
//            ESP_LOGI(TAG, "Signal on GPIO 12 is %s!", signal == 1 ? "ON" : "OFF");
//        }
//        vTaskDelay(1);
//    }

    int i2c_master_port = 0;
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,         // select SDA GPIO specific to your project
            .scl_io_num = 22,         // select SCL GPIO specific to your project
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000  // select frequency specific to your project
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t packet1[2] = {0x0B, 0X01}; // регистр set/reset выставляем в set
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            i2c_master_port,
            COMPASS_I2C_ADDRESS,
            packet1, 2,
            10));
    uint8_t packet2[2] = {0x09, 0b00010001}; // OSR = 512, Full Scale Range = 8 Gauss, ODR = 10Hz, continuous measurement mode
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            i2c_master_port,
            COMPASS_I2C_ADDRESS,
            packet2, 2,
            10));
    ESP_LOGI(TAG, "Compass set successfully");

    while (1) {
        uint8_t status_register = 0x06;
        uint8_t response;
        ESP_ERROR_CHECK(i2c_master_write_read_device(
                i2c_master_port,
                COMPASS_I2C_ADDRESS,
                &status_register, 1,
                &response, 1,
                10));

        ESP_LOGI(TAG, "status is %d", response);
        if (!(response & 1)) {
            ESP_LOGI(TAG, "not ready");
            vTaskDelay(250);
            continue;
        }
        uint8_t x_lsb_register = 0x00;
        uint8_t x_response[6];
        ESP_ERROR_CHECK(i2c_master_write_read_device(
                i2c_master_port,
                COMPASS_I2C_ADDRESS,
                &x_lsb_register, 1,
                x_response, 6,
                10));
        ESP_LOGI(TAG, "x = [%d][%d] y = [%d][%d] z = [%d][%d]", x_response[0], x_response[1],
                 x_response[2], x_response[3],
                 x_response[4], x_response[5]);
        vTaskDelay(250);
    }
}