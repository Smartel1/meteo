#include "qmc5883l.h"
#include <driver/i2c.h>
#include <math.h>

void qmc5883l_init(qmc5883l_settings *settings) {
    qmc5883l_init_all(settings, NULL, NULL, NULL, NULL);
}

void qmc5883l_init_all(qmc5883l_settings *settings, qmc5883l_mode_t *mode, qmc5883l_rate_t *rate, qmc5883l_range_t *range,
                       qmc5883l_oversample_t *os) {
    qmc5883l_mode_t m = QMC5883L_CONFIG_CONT;
    if (mode != NULL) {
        m = *mode;
    }
    qmc5883l_rate_t rt = QMC5883L_CONFIG_200HZ;
    if (rate != NULL) {
        rt = *rate;
    }
    qmc5883l_range_t rg = QMC5883L_CONFIG_8GAUSS;
    if (range != NULL) {
        rg = *range;
    }
    qmc5883l_oversample_t osmpl = QMC5883L_CONFIG_OS512;
    if (os != NULL) {
        osmpl = *os;
    }
    uint8_t config = ((uint8_t) (osmpl) << 6) | ((uint8_t) (rg) << 4) | ((uint8_t) (rt) << 2) | (uint8_t) (m);
    qmc5883l_reset(settings);
    uint8_t packet[2] = {QMC5883L_CONFIG, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}

bool qmc5883l_data_ready(qmc5883l_settings *settings) {
    uint8_t res = 0;

    uint8_t packet[1] = {QMC5883L_STATUS};
    ESP_ERROR_CHECK(i2c_master_write_read_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 1,
            &res, 1,
            10));

    if (res & QMC5883L_STATUS_DRDY) {
        return true;
    }
    return false;
}

bool qmc5883l_get_data(qmc5883l_settings *settings, qmc5883l_data_t *data) {
    //check if device is operating and if data is ready;
    if (qmc5883l_get_mode(settings) == QMC5883L_CONFIG_STANDBY) {
        return false;
    }
    //read data from device
    uint8_t buf[6];
    uint8_t packet[1] = {QMC5883L_X_LSB};
    ESP_ERROR_CHECK(i2c_master_write_read_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 1,
            buf, 6,
            10));
    data->x = (uint16_t) buf[QMC5883L_X_MSB] << 8 | buf[QMC5883L_X_LSB];
    data->y = (uint16_t) buf[QMC5883L_Y_MSB] << 8 | buf[QMC5883L_Y_LSB];
    data->z = (uint16_t) buf[QMC5883L_Z_MSB] << 8 | buf[QMC5883L_Z_LSB];
    return true;
}

bool qmc5883l_get_azimuth(qmc5883l_settings *settings, int16_t *angle) {
    qmc5883l_data_t compass_data;
    qmc5883l_get_data(settings, &compass_data);
    int a = atan2((compass_data.x + settings->x_offset) * settings->x_scale,
                  (compass_data.z + settings->y_offset) * settings->y_scale) * 180.0 / 3.14;
    a = a < 0 ? 360 + a : a;
    *angle=a;
    return true;
}

bool qmc5883l_get_temp(qmc5883l_settings *settings, int16_t *temp) {
    //check if device is operating and if data is ready;
    if (qmc5883l_get_mode(settings) == QMC5883L_CONFIG_STANDBY) {
        return false;
    }
    //read data from device
    uint8_t buf[2];
    uint8_t register_address = QMC5883L_TEMP_LSB;
    ESP_ERROR_CHECK(i2c_master_write_read_device(
            settings->port,
            QMC5883L_ADDR,
            &register_address, 1,
            buf, 2,
            10));
    *temp = (int16_t) (buf[1] << 8 | buf[0]) / 100 + 34;
    return true;
}

void qmc5883l_reset(qmc5883l_settings *settings) {
    uint8_t packet[2] = {QMC5883L_RESET, 1};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}

uint8_t qmc5883l_get_config(qmc5883l_settings *settings) {
    uint8_t res = 0;
    uint8_t packet[1] = {QMC5883L_CONFIG};
    ESP_ERROR_CHECK(i2c_master_write_read_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 1,
            &res, 1,
            10));
    return res;
}

qmc5883l_mode_t qmc5883l_get_mode(qmc5883l_settings *settings) {
    uint8_t config = qmc5883l_get_config(settings);
    qmc5883l_mode_t mode = config & 0b00000011;
    return mode;
}

void qmc5883l_set_mode(qmc5883l_settings *settings, qmc5883l_mode_t mode) {
    uint8_t config = qmc5883l_get_config(settings);
    config = (config & 0b11111100) | mode;
    uint8_t packet[2] = {QMC5883L_CONFIG, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}

qmc5883l_rate_t qmc5883l_get_rate(qmc5883l_settings *settings) {
    uint8_t config = qmc5883l_get_config(settings);
    qmc5883l_rate_t rate = (config & 0b00001100) >> 2;
    return rate;
}

void qmc5883l_set_rate(qmc5883l_settings *settings, qmc5883l_rate_t rate) {
    uint8_t config = qmc5883l_get_config(settings);
    config = (config & 0b11110011) | ((uint8_t) rate << 2);
    uint8_t packet[2] = {QMC5883L_CONFIG, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}

qmc5883l_range_t qmc5883l_get_range(qmc5883l_settings *settings) {
    uint8_t config = qmc5883l_get_config(settings);
    qmc5883l_range_t range = (config & 0b00110000) >> 4;
    return range;
}

void qmc5883l_set_range(qmc5883l_settings *settings, qmc5883l_range_t range) {
    uint8_t config = qmc5883l_get_config(settings);
    config = (config & 0b11001111) | ((uint8_t) range << 4);
    uint8_t packet[2] = {QMC5883L_CONFIG, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}

qmc5883l_oversample_t qmc5883l_get_oversample(qmc5883l_settings *settings) {
    uint8_t config = qmc5883l_get_config(settings);
    qmc5883l_oversample_t oversample = (config & 0b11000000) >> 6;
    return oversample;
}

void qmc5883l_set_oversample(qmc5883l_settings *settings, qmc5883l_oversample_t oversample) {
    uint8_t config = qmc5883l_get_config(settings);
    config = (config & 0b00111111) | ((uint8_t) oversample << 6);
    uint8_t packet[2] = {QMC5883L_CONFIG, config};
    ESP_ERROR_CHECK(i2c_master_write_to_device(
            settings->port,
            QMC5883L_ADDR,
            packet, 2,
            10));
}