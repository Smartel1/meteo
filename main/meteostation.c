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
#include "bmx280.h"

static const char *TAG = "meteo";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define HALL_GPIO GPIO_NUM_12
#define I2C_PORT 0
#define GPIO_SDA GPIO_NUM_21
#define GPIO_SCL GPIO_NUM_22
#define IP5306_ADDR 0x75 // Адрес устройства IP5306
#define IP5306_REG_SYS_CTL0 0x00 // Регистр IP5306_SYS_CTL0

#define DEVICE_ID 1
#define SERVER_ADDRESS "193.124.125.33"
#define MEASURING_INTERVAL 60000000 // 1 minute - todo change to 5 min

#define portTICK_RATE_MS     ( (TickType_t) 1000 / configTICK_RATE_HZ )

bmx280_t *bmx280;

// калибровки компаса
static int X_OFFSET = -191;
static float X_SCALE = 0.87f;
static int Z_OFFSET = 53;
static float Z_SCALE = 1.17f;

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

void configure_ip5306() {
    uint8_t config = 0b10111110;
    uint8_t packet[2] = {IP5306_REG_SYS_CTL0, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            I2C_PORT,
            IP5306_ADDR,
            packet, 2,
            10));
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

static void calibrate_compass(void) {
    qmc5883l_calibrate(&compass_settings);
}

static uint8_t get_azimuth(void) {
    int16_t azimuth;
    qmc5883l_get_azimuth(&compass_settings, &azimuth);
    ESP_LOGI(TAG, "azimuth = %d", azimuth);
    return azimuth;
}

static float get_voltage(void) {
    float voltage = (float) adc1_get_raw(ADC1_CHANNEL_7) / 386.0;
    ESP_LOGI(TAG, "voltage = %.1f", voltage);
    return voltage;
}

static void init_i2c(void) {
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_SDA,         // select SDA GPIO specific to your project
            .scl_io_num = GPIO_SCL,         // select SCL GPIO specific to your project
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");
}

static void init_compass(void) {
    qmc5883l_settings compass_settings_obj = {
            .port = I2C_PORT,
            .x_offset = X_OFFSET,
            .x_scale = X_SCALE,
            .z_offset = Z_OFFSET,
            .z_scale = Z_SCALE
    };
    compass_settings = compass_settings_obj;
    qmc5883l_init(&compass_settings);
    ESP_LOGI(TAG, "Compass set successfully");
}

static void init_bmp280(void) {
    bmx280 = bmx280_create(I2C_PORT);
    if (!bmx280) {
        ESP_LOGE("test", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (bmx280_isSampling(bmx280));
}

static void get_pressure_and_temperature(int *pressure, int *temperature) {
    float btemp = 0, bpres = 0, hum = 0;
    ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &btemp, &bpres, &hum));

    ESP_LOGI(TAG, "Read Values: temp = %f, pres = %f", btemp, bpres);

    *pressure = bpres / 133.3; // Преобразование из Pa в мм рт ст
    *temperature = btemp;
}

void send_metrics(float wind_speed, int azimuth, int temperature, float voltage, int pressure) {
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
    sprintf(request_body, "%.1f %d %d %.1f %d", wind_speed, azimuth, temperature, voltage, pressure);
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

    init_i2c();
    blink();

    configure_ip5306();
    blink();

    init_compass();
// Use this function to calibrate compass (once before meteostation installation).
// Uncomment this line, run the code and rotate the compass in XZ plane (couple of rounds).
// Get offsets and scales from console and put them in X_OFFSET, Z_OFFSET, X_SCALE and Z_SCALE constants. Thats it!
//    calibrate_compass();
    blink();

    init_bmp280();
    blink();

    configureUART();
    blink();

    turnOnSim800l();
    blink();

    float wind_speed = get_wind_speed();
    uint8_t azimuth = get_azimuth();
    float voltage = get_voltage();

    int pressure, temperature;
    get_pressure_and_temperature(&pressure, &temperature);

    send_metrics(wind_speed, azimuth, temperature, voltage, pressure);
    turnOffSim800l();

    esp_sleep_enable_timer_wakeup(MEASURING_INTERVAL);
    ESP_LOGI(TAG, "Going down for 5 min");
    esp_deep_sleep_start();
}