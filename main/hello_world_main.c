#include <stdio.h>
#include <esp_timer.h>
#include <driver/i2c.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "qmc5883l.c"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22

static uint8_t s_led_state = 0;
static uint8_t gpio12state = 0;
static int64_t prevTickMs = 0;
static uint8_t COMPASS_I2C_ADDRESS = 0x0D;

static void blink_led(void) {
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void) {

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
            .sda_io_num = GPIO_SDA,         // select SDA GPIO specific to your project
            .scl_io_num = GPIO_SCL,         // select SCL GPIO specific to your project
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");

    i2c_dev_t compass_address = {
            .bus = i2c_master_port,
            .addr = COMPASS_I2C_ADDRESS
    };
    qmc5883l_init(&compass_address);
    ESP_LOGI(TAG, "Compass set successfully");

    while (1) {
        qmc5883l_data_t compass_data;
        qmc5883l_get_data(&compass_address, &compass_data);
        ESP_LOGI(TAG, "x = [%d] y = [%d] z = [%d]", compass_data.x, compass_data.y, compass_data.z);
        uint16_t temp;
        qmc5883l_get_temp(&compass_address, &temp);
        ESP_LOGI(TAG, "temp = %d", temp);
        vTaskDelay(250);
    }
}