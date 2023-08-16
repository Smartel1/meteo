#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include <string.h>
#include <driver/adc.h>
#include <esp_sleep.h>
#include "qmc5883l.h"
#include "sim800l.h"

static const char *TAG = "meteo";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define HALL_GPIO GPIO_NUM_12
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22

#define DEVICE_ID 1
#define SERVER_ADDRESS "193.124.125.33"
#define MEASURING_INTERVAL 60000000 // 1 minute - todo change to 5 min

#define portTICK_RATE_MS     ( (TickType_t) 1000 / configTICK_RATE_HZ )

// калибровки компаса
static uint8_t X_OFFSET = 150;
static float X_SCALE = 3.2f;
static uint8_t Y_OFFSET = 185;
static float Y_SCALE = 3.03f;

static uint8_t hall_transitions = 0;

static qmc5883l_settings compass_settings;

static void init_led(void) {
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink(void) {
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(5);
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(5);
}

void increment_hall_transitions(void* arg) {
    hall_transitions++;
    static int led = 0;
    gpio_set_level(BLINK_GPIO, !led);
    led = !led;
}

static void init_hall_sensor(void) {
    gpio_reset_pin(HALL_GPIO);
    gpio_set_direction(HALL_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HALL_GPIO, GPIO_PULLUP_ONLY);
    // ISR is being invoked 2 times a round
    gpio_set_intr_type(HALL_GPIO, GPIO_INTR_ANYEDGE);
    gpio_intr_enable(HALL_GPIO);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(HALL_GPIO, increment_hall_transitions, NULL);
}

static float get_wind_speed(void) {
    ESP_LOGI(TAG, "Measuring wind speed");
    uint8_t prev_hall_transitions = hall_transitions;
    vTaskDelay(5000 / portTICK_RATE_MS);
    uint8_t rounds_count = (hall_transitions - prev_hall_transitions) / 4; // 2 on and 2 offs per round
    ESP_LOGI(TAG, "Rounds made in 5 sec: %d", rounds_count);
    return rounds_count; //todo подставить коэффициент
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

static float get_voltage(void) {
    float voltage = (float) adc1_get_raw(ADC1_CHANNEL_7) / 386.0;
    ESP_LOGI(TAG, "voltage = %.1f", voltage);
    return voltage;
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
    char url_cmd[100];
    sprintf(url_cmd, "AT+HTTPPARA=\"URL\",\"http://%s/spots/%d/metrics\"", SERVER_ADDRESS, DEVICE_ID);
    sendCommand(url_cmd);
    sendCommand("AT+HTTPPARA=\"CONTENT\",\"text/plain\"");
    char request_body[100];
    sprintf(request_body, "%.1f %d %d %.1f", wind_speed, azimuth, temperature, voltage);
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
    blink();

    init_hall_sensor();
    blink();

    init_compass();
    blink();

    configureUART();
    blink();

    turnOnSim800l();
    blink();

    float wind_speed = get_wind_speed();
    uint8_t azimuth = get_azimuth();
    uint8_t temp = get_temp();
    float voltage = get_voltage();

    send_metrics(wind_speed, azimuth, temp, voltage);
    turnOffSim800l();

    esp_sleep_enable_timer_wakeup(MEASURING_INTERVAL);
    ESP_LOGI(TAG, "Going down for 5 min");
    esp_deep_sleep_start();
}