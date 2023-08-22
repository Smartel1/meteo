#include <string.h>
#include "esp_log.h"
#include "sim800l.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

///Pin Config
#define TXD                  (GPIO_NUM_27)
#define RXD                  (GPIO_NUM_26)
#define RESET_PIN            (GPIO_NUM_5)
#define PWR_ON_PIN           (GPIO_NUM_23)
#define PWR_KEY_PIN          (GPIO_NUM_4)

#define BUF_SIZE             (1024)

#define UART_NUM             UART_NUM_1
#define UART_BAUD            38400


static const char *TAG = "[SIM800L DRIVER]";

void configureUART() {
    gpio_set_direction(TXD, GPIO_MODE_OUTPUT);
    gpio_set_direction(RXD, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RXD, GPIO_PULLUP_ONLY);

    uart_config_t uart_config = {
            .baud_rate = UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    ///Reset Pin -> Active Low Pin
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PIN, 1);
}

void turnOnSim800l() {
    ESP_LOGI(TAG, "Turning on SIM800L");
    gpio_set_direction(PWR_ON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_ON_PIN, 1);
    gpio_set_direction(PWR_KEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_KEY_PIN, 1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(PWR_KEY_PIN, 0);
    vTaskDelay(1100 / portTICK_PERIOD_MS);
    gpio_set_level(PWR_KEY_PIN, 1);
    vTaskDelay(3500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "SIM800L is turned ON!");
}

void turnOffSim800l() {
    gpio_set_level(PWR_KEY_PIN, 0);
    vTaskDelay(1100 / portTICK_PERIOD_MS);
    gpio_set_level(PWR_KEY_PIN, 1);

    ESP_LOGI(TAG, "SIM800L is turned OFF!");
}

void sendCommand(char *cmd) {
    uart_flush(uart_num);
    uart_write_bytes(UART_NUM, (const char *) cmd, strlen(cmd));
    uart_wait_tx_done(UART_NUM, 100 / portTICK_PERIOD_MS);
    uart_write_bytes(UART_NUM, "\r\n", 2);
    uart_wait_tx_done(UART_NUM, 150 / portTICK_PERIOD_MS);

    char response_buf[100] = {""};
    int response_len = uart_read_bytes(UART_NUM, (uint8_t *) response_buf, sizeof response_buf,
                                       2000 / portTICK_PERIOD_MS);
    response_buf[response_len] = '\0';
    ESP_LOGI(TAG, "len = %d", response_len);
    ESP_LOGI(TAG, "%s", response_buf);
}


