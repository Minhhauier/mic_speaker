#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <config_parameter.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── MPU6050 I2C Address ─────────────────────────────────────── */
#define MPU6050_ADDR_LOW    0x68   // AD0 = GND
#define MPU6050_ADDR_HIGH   0x69   // AD0 = VCC

/* ─── Register Map ───────────────────────────────────────────── */
#define MPU6050_REG_SELF_TEST_X     0x0D
#define MPU6050_REG_SELF_TEST_Y     0x0E
#define MPU6050_REG_SELF_TEST_Z     0x0F
#define MPU6050_REG_SELF_TEST_A     0x10
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75
// extern variable mpu6050_handle_t mpu;

/* ─── Accelerometer Full-Scale Range ─────────────────────────── */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,   // ±2g  — LSB: 16384 LSB/g
    MPU6050_ACCEL_FS_4G  = 0x08,   // ±4g  — LSB: 8192  LSB/g
    MPU6050_ACCEL_FS_8G  = 0x10,   // ±8g  — LSB: 4096  LSB/g
    MPU6050_ACCEL_FS_16G = 0x18,   // ±16g — LSB: 2048  LSB/g
} mpu6050_accel_fs_t;

/* ─── Gyroscope Full-Scale Range ─────────────────────────────── */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0x00,  // ±250°/s  — LSB: 131.0
    MPU6050_GYRO_FS_500DPS  = 0x08,  // ±500°/s  — LSB: 65.5
    MPU6050_GYRO_FS_1000DPS = 0x10,  // ±1000°/s — LSB: 32.8
    MPU6050_GYRO_FS_2000DPS = 0x18,  // ±2000°/s — LSB: 16.4
} mpu6050_gyro_fs_t;

/* ─── DLPF Bandwidth ─────────────────────────────────────────── */
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} mpu6050_dlpf_t;

/* ─── Raw Data Structs ───────────────────────────────────────── */
typedef struct {
    int16_t x, y, z;
} mpu6050_raw_t;

typedef struct {
    float x, y, z;           // Accel in g, Gyro in °/s
} mpu6050_data_t;

/* ─── Device Handle ─────────────────────────────────────────── */
typedef struct {
    i2c_master_dev_handle_t dev_handle;
    float accel_scale;        // LSB per g
    float gyro_scale;         // LSB per °/s
} mpu6050_handle_t;

/* ─── Configuration ──────────────────────────────────────────── */
typedef struct {
    uint8_t            i2c_addr;
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_dlpf_t     dlpf;
    uint8_t            sample_rate_div;  // Output rate = 1kHz / (1 + div)
} mpu6050_config_t;

/* ─── Default Config Macro ───────────────────────────────────── */
#define MPU6050_DEFAULT_CONFIG() { \
    .i2c_addr       = MPU6050_ADDR_LOW, \
    .accel_fs       = MPU6050_ACCEL_FS_2G, \
    .gyro_fs        = MPU6050_GYRO_FS_250DPS, \
    .dlpf           = MPU6050_DLPF_44HZ, \
    .sample_rate_div = 9, \
}

/* ─── API ────────────────────────────────────────────────────── */
extern mpu6050_data_t accel;
/**
 * @brief  Khởi tạo MPU6050 trên I2C master bus
 * @param  bus_handle   I2C master bus handle (đã khởi tạo)
 * @param  config       Cấu hình cảm biến
 * @param  out_handle   Handle trả về
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle,
                       const mpu6050_config_t *config,
                       mpu6050_handle_t *out_handle);

/** @brief  Reset và giải phóng MPU6050 */
esp_err_t mpu6050_deinit(mpu6050_handle_t *handle);

/** @brief  Đọc WHO_AM_I (nên = 0x68) để kiểm tra kết nối */
esp_err_t mpu6050_who_am_i(const mpu6050_handle_t *handle, uint8_t *who);

/** @brief  Đọc dữ liệu thô Accel (đơn vị LSB) */
esp_err_t mpu6050_read_accel_raw(const mpu6050_handle_t *handle, mpu6050_raw_t *out);

/** @brief  Đọc dữ liệu thô Gyro (đơn vị LSB) */
esp_err_t mpu6050_read_gyro_raw(const mpu6050_handle_t *handle, mpu6050_raw_t *out);

/** @brief  Đọc nhiệt độ thô */
esp_err_t mpu6050_read_temp_raw(const mpu6050_handle_t *handle, int16_t *out);

/** @brief  Đọc Accel đã chuyển đổi sang đơn vị g */
esp_err_t mpu6050_read_accel(const mpu6050_handle_t *handle, mpu6050_data_t *out);

/** @brief  Đọc Gyro đã chuyển đổi sang đơn vị °/s */
esp_err_t mpu6050_read_gyro(const mpu6050_handle_t *handle, mpu6050_data_t *out);

/** @brief  Đọc nhiệt độ theo °C */
esp_err_t mpu6050_read_temp(const mpu6050_handle_t *handle, float *celsius);

/** @brief  Đọc tất cả dữ liệu cùng lúc (burst read 14 bytes) */
esp_err_t mpu6050_read_all(const mpu6050_handle_t *handle,
                           mpu6050_data_t *accel,
                           mpu6050_data_t *gyro,
                           float *temp_c);

/** @brief  Self-test cơ bản — trả về ESP_OK nếu pass */
esp_err_t mpu6050_self_test(mpu6050_handle_t *handle);
void mpu6050_start();
void mpu6050_task(void *arg);
#ifdef __cplusplus
}
#endif