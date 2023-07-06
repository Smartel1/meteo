#include <stdint.h>
#include <stdbool.h>

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
typedef enum {
    QMC5883L_X_LSB = 0,
    QMC5883L_X_MSB = 1,
    QMC5883L_Y_LSB = 2,
    QMC5883L_Y_MSB = 3,
    QMC5883L_Z_LSB = 4,
    QMC5883L_Z_MSB = 5,
    QMC5883L_STATUS = 6,
    QMC5883L_TEMP_LSB = 7,
    QMC5883L_TEMP_MSB = 8,
    QMC5883L_CONFIG = 9,
    QMC5883L_CONFIG2 = 10,
    QMC5883L_RESET = 11,
    QMC5883L_RESERVED = 12,
    QMC5883L_CHIP_ID = 13
} qmc5883l_register_t;

/* Bit values for the STATUS register */
typedef enum {
    QMC5883L_STATUS_DRDY = 1,
    QMC5883L_STATUS_OVL = 2,
    QMC5883L_STATUS_DOR = 4
} qmc5883l_state_t;

/* Oversampling values for the CONFIG register */
typedef enum {
    QMC5883L_CONFIG_OS512 = 0,
    QMC5883L_CONFIG_OS256 = 1,
    QMC5883L_CONFIG_OS128 = 2,
    QMC5883L_CONFIG_OS64  = 3
} qmc5883l_oversample_t;

/* Range values for the CONFIG register */
typedef enum {
    QMC5883L_CONFIG_2GAUSS = 0,
    QMC5883L_CONFIG_8GAUSS = 1
} qmc5883l_range_t;

/* Rate values for the CONFIG register */
typedef enum {
    QMC5883L_CONFIG_10HZ   = 0,
    QMC5883L_CONFIG_50HZ   = 1,
    QMC5883L_CONFIG_100HZ  = 2,
    QMC5883L_CONFIG_200HZ  = 3
} qmc5883l_rate_t;

/* Mode values for the CONFIG register */
typedef enum {
    QMC5883L_CONFIG_STANDBY = 0,
    QMC5883L_CONFIG_CONT = 1
} qmc5883l_mode_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883l_data_t;

typedef struct {
    uint8_t port; // номер логического порта
    uint8_t x_offset; // офсет по Х (отклонение среднего значения от нуля)
    float x_scale; // коэфф. для X (для приведения всех осей к одному модулю)
    uint8_t y_offset; // офсет по Y (отклонение среднего значения от нуля)
    float y_scale; // коэфф. для Y (для приведения всех осей к одному модулю)
} qmc5883l_settings;

void qmc5883l_init(qmc5883l_settings *compass_address);
void qmc5883l_init_all(qmc5883l_settings *compass_address, qmc5883l_mode_t *mode, qmc5883l_rate_t *rate, qmc5883l_range_t *range, qmc5883l_oversample_t *sample);
bool qmc5883l_data_ready(qmc5883l_settings *compass_address);
void qmc5883l_reset(qmc5883l_settings *compass_address);
uint8_t qmc5883l_get_config(qmc5883l_settings *compass_address);

qmc5883l_mode_t qmc5883l_get_mode(qmc5883l_settings *compass_address);
void qmc5883l_set_mode(qmc5883l_settings *compass_address, qmc5883l_mode_t mode);

qmc5883l_oversample_t qmc5883l_get_oversample(qmc5883l_settings *compass_address);
void qmc5883l_set_oversample(qmc5883l_settings *compass_address, qmc5883l_oversample_t os);

qmc5883l_range_t qmc5883l_get_range(qmc5883l_settings *compass_address);
void qmc5883l_set_range(qmc5883l_settings *compass_address, qmc5883l_range_t range);

qmc5883l_rate_t qmc5883l_get_rate(qmc5883l_settings *compass_address);
void qmc5883l_set_rate(qmc5883l_settings *compass_address, qmc5883l_rate_t rate);

bool qmc5883l_get_data(qmc5883l_settings *compass_address, qmc5883l_data_t* data);
bool qmc5883l_get_azimuth(qmc5883l_settings *compass_address, int16_t *angle);
bool qmc5883l_get_temp(qmc5883l_settings *compass_address, int16_t *temp);