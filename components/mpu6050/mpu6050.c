#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include "esp_check.h".

static const char *TAG = "MPU6050";
mpu6050_data_t accel;
#define I2C_TIMEOUT_MS      100
#define MPU6050_WHO_AM_I_VAL 0x68
mpu6050_handle_t mpu;
/* ─── Internal helpers ───────────────────────────────────────── */

static esp_err_t mpu_write_reg(const mpu6050_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(h->dev_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t mpu_read_reg(const mpu6050_handle_t *h, uint8_t reg,
                               uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(h->dev_handle, &reg, 1,
                                       data, len, I2C_TIMEOUT_MS);
}

static inline int16_t combine(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}

/* ─── Public API ─────────────────────────────────────────────── */

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle,
                       const mpu6050_config_t *config,
                       mpu6050_handle_t *out_handle)
{
    ESP_LOGI(TAG, "Initializing MPU6050 at addr 0x%02X", config->i2c_addr);

    /* Add device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = config->i2c_addr,
        .scl_speed_hz    = 400000,  // 400 kHz Fast Mode
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(bus_handle, &dev_cfg, &out_handle->dev_handle),
        TAG, "Failed to add device to I2C bus");

    /* Wake up: clear SLEEP bit in PWR_MGMT_1, use PLL with X-Gyro clock */
    ESP_RETURN_ON_ERROR(mpu_write_reg(out_handle, MPU6050_REG_PWR_MGMT_1, 0x01),
                        TAG, "PWR_MGMT_1 write failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Verify WHO_AM_I */
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu6050_who_am_i(out_handle, &who),
                        TAG, "WHO_AM_I read failed");
    if (who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, MPU6050_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "WHO_AM_I OK: 0x%02X", who);

    /* Sample rate divider */
    ESP_RETURN_ON_ERROR(mpu_write_reg(out_handle, MPU6050_REG_SMPLRT_DIV,
                                       config->sample_rate_div),
                        TAG, "SMPLRT_DIV write failed");

    /* DLPF config */
    ESP_RETURN_ON_ERROR(mpu_write_reg(out_handle, MPU6050_REG_CONFIG,
                                       (uint8_t)config->dlpf),
                        TAG, "CONFIG write failed");

    /* Gyro full-scale */
    ESP_RETURN_ON_ERROR(mpu_write_reg(out_handle, MPU6050_REG_GYRO_CONFIG,
                                       (uint8_t)config->gyro_fs),
                        TAG, "GYRO_CONFIG write failed");

    /* Accel full-scale */
    ESP_RETURN_ON_ERROR(mpu_write_reg(out_handle, MPU6050_REG_ACCEL_CONFIG,
                                       (uint8_t)config->accel_fs),
                        TAG, "ACCEL_CONFIG write failed");

    /* Compute scale factors */
    const float accel_lsb[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    const float gyro_lsb[]  = {131.0f, 65.5f, 32.8f, 16.4f};
    uint8_t accel_idx = (config->accel_fs >> 3) & 0x03;
    uint8_t gyro_idx  = (config->gyro_fs  >> 3) & 0x03;
    out_handle->accel_scale = accel_lsb[accel_idx];
    out_handle->gyro_scale  = gyro_lsb[gyro_idx];

    ESP_LOGI(TAG, "Init OK | Accel ±%dg (%.0f LSB/g) | Gyro ±%d°/s (%.1f LSB/°/s)",
             2 << accel_idx, out_handle->accel_scale,
             250 << gyro_idx,  out_handle->gyro_scale);
    return ESP_OK;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t *handle)
{
    /* Put sensor to sleep */
    mpu_write_reg(handle, MPU6050_REG_PWR_MGMT_1, 0x40);
    esp_err_t ret = i2c_master_bus_rm_device(handle->dev_handle);
    handle->dev_handle = NULL;
    return ret;
}

esp_err_t mpu6050_who_am_i(const mpu6050_handle_t *handle, uint8_t *who)
{
    return mpu_read_reg(handle, MPU6050_REG_WHO_AM_I, who, 1);
}

esp_err_t mpu6050_read_accel_raw(const mpu6050_handle_t *handle, mpu6050_raw_t *out)
{
    uint8_t buf[6];
    ESP_RETURN_ON_ERROR(mpu_read_reg(handle, MPU6050_REG_ACCEL_XOUT_H, buf, 6),
                        TAG, "Accel raw read failed");
    out->x = combine(buf[0], buf[1]);
    out->y = combine(buf[2], buf[3]);
    out->z = combine(buf[4], buf[5]);
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro_raw(const mpu6050_handle_t *handle, mpu6050_raw_t *out)
{
    uint8_t buf[6];
    ESP_RETURN_ON_ERROR(mpu_read_reg(handle, MPU6050_REG_GYRO_XOUT_H, buf, 6),
                        TAG, "Gyro raw read failed");
    out->x = combine(buf[0], buf[1]);
    out->y = combine(buf[2], buf[3]);
    out->z = combine(buf[4], buf[5]);
    return ESP_OK;
}

esp_err_t mpu6050_read_temp_raw(const mpu6050_handle_t *handle, int16_t *out)
{
    uint8_t buf[2];
    ESP_RETURN_ON_ERROR(mpu_read_reg(handle, MPU6050_REG_TEMP_OUT_H, buf, 2),
                        TAG, "Temp raw read failed");
    *out = combine(buf[0], buf[1]);
    return ESP_OK;
}

esp_err_t mpu6050_read_accel(const mpu6050_handle_t *handle, mpu6050_data_t *out)
{
    mpu6050_raw_t raw;
    ESP_RETURN_ON_ERROR(mpu6050_read_accel_raw(handle, &raw), TAG, "");
    out->x = raw.x / handle->accel_scale;
    out->y = raw.y / handle->accel_scale;
    out->z = raw.z / handle->accel_scale;
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(const mpu6050_handle_t *handle, mpu6050_data_t *out)
{
    mpu6050_raw_t raw;
    ESP_RETURN_ON_ERROR(mpu6050_read_gyro_raw(handle, &raw), TAG, "");
    out->x = raw.x / handle->gyro_scale;
    out->y = raw.y / handle->gyro_scale;
    out->z = raw.z / handle->gyro_scale;
    return ESP_OK;
}

esp_err_t mpu6050_read_temp(const mpu6050_handle_t *handle, float *celsius)
{
    int16_t raw;
    ESP_RETURN_ON_ERROR(mpu6050_read_temp_raw(handle, &raw), TAG, "");
    /* Formula from datasheet: Temp(°C) = raw/340 + 36.53 */
    *celsius = (float)raw / 340.0f + 36.53f;
    return ESP_OK;
}

esp_err_t mpu6050_read_all(const mpu6050_handle_t *handle,
                           mpu6050_data_t *accel,
                           mpu6050_data_t *gyro,
                           float *temp_c)
{
    /* Burst read: ACCEL (6) + TEMP (2) + GYRO (6) = 14 bytes */
    uint8_t buf[14];
    ESP_RETURN_ON_ERROR(mpu_read_reg(handle, MPU6050_REG_ACCEL_XOUT_H, buf, 14),
                        TAG, "Burst read failed");

    accel->x = combine(buf[0],  buf[1])  / handle->accel_scale;
    accel->y = combine(buf[2],  buf[3])  / handle->accel_scale;
    accel->z = combine(buf[4],  buf[5])  / handle->accel_scale;

    int16_t raw_temp = combine(buf[6], buf[7]);
    *temp_c = (float)raw_temp / 340.0f + 36.53f;

    gyro->x  = combine(buf[8],  buf[9])  / handle->gyro_scale;
    gyro->y  = combine(buf[10], buf[11]) / handle->gyro_scale;
    gyro->z  = combine(buf[12], buf[13]) / handle->gyro_scale;

    return ESP_OK;
}

esp_err_t mpu6050_self_test(mpu6050_handle_t *handle)
{
    ESP_LOGI(TAG, "Running self-test...");

    /* 1. Lưu config hiện tại */
    uint8_t saved_gyro_cfg, saved_accel_cfg;
    mpu_read_reg(handle, MPU6050_REG_GYRO_CONFIG,  &saved_gyro_cfg, 1);
    mpu_read_reg(handle, MPU6050_REG_ACCEL_CONFIG, &saved_accel_cfg, 1);

    /* 2. Set full-scale: Gyro 250°/s, Accel 8g (yêu cầu self-test) */
    mpu_write_reg(handle, MPU6050_REG_GYRO_CONFIG,  0x00);
    mpu_write_reg(handle, MPU6050_REG_ACCEL_CONFIG, 0x10);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Đọc output không có self-test */
    mpu6050_raw_t gyro_no_st, accel_no_st;
    mpu6050_read_gyro_raw(handle, &gyro_no_st);
    mpu6050_read_accel_raw(handle, &accel_no_st);

    /* 4. Bật self-test bits (XA YA ZA XG YG ZG) */
    mpu_write_reg(handle, MPU6050_REG_GYRO_CONFIG,  0xE0);  // bits 7:5
    mpu_write_reg(handle, MPU6050_REG_ACCEL_CONFIG, 0xF0);  // bits 7:5 + FS=2g
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 5. Đọc output khi self-test */
    mpu6050_raw_t gyro_st, accel_st;
    mpu6050_read_gyro_raw(handle, &gyro_st);
    mpu6050_read_accel_raw(handle, &accel_st);

    /* 6. Self-test response (STR) */
    int16_t str_gx = gyro_st.x  - gyro_no_st.x;
    int16_t str_gy = gyro_st.y  - gyro_no_st.y;
    int16_t str_gz = gyro_st.z  - gyro_no_st.z;
    int16_t str_ax = accel_st.x - accel_no_st.x;
    int16_t str_ay = accel_st.y - accel_no_st.y;
    int16_t str_az = accel_st.z - accel_no_st.z;

    ESP_LOGI(TAG, "STR Gyro  X=%d Y=%d Z=%d", str_gx, str_gy, str_gz);
    ESP_LOGI(TAG, "STR Accel X=%d Y=%d Z=%d", str_ax, str_ay, str_az);

    /* 7. Phục hồi cấu hình cũ */
    mpu_write_reg(handle, MPU6050_REG_GYRO_CONFIG,  saved_gyro_cfg);
    mpu_write_reg(handle, MPU6050_REG_ACCEL_CONFIG, saved_accel_cfg);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 8. Kiểm tra: STR khác 0 là có phản hồi self-test */
    if (str_gx == 0 && str_gy == 0 && str_gz == 0 &&
        str_ax == 0 && str_ay == 0 && str_az == 0) {
        ESP_LOGE(TAG, "Self-test FAILED: no response");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Self-test PASSED");
    return ESP_OK;
}

void mpu6050_start(){
    //  print_sep('*', 50);
    // ESP_LOGI(TAG, "  MPU6050 Test Suite — ESP-IDF");
    // ESP_LOGI(TAG, "  SCL=GPIO%d  SDA=GPIO%d  400kHz",
    //          I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    // print_sep('*', 50);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ── Khởi tạo I2C Master Bus ── */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_MASTER_PORT,
        .sda_io_num        = I2C_MASTER_SDA_IO,
        .scl_io_num        = I2C_MASTER_SCL_IO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C Master bus khởi tạo OK");

    /* ── Khởi tạo MPU6050 ── */
    mpu6050_config_t mpu_cfg = MPU6050_DEFAULT_CONFIG();
  

    esp_err_t ret = mpu6050_init(bus_handle, &mpu_cfg, &mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Khởi tạo MPU6050 thất bại: %s", esp_err_to_name(ret));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void mpu6050_task(void *arg)
{
    mpu6050_start();
    while (1)
    {
        if (mpu6050_read_accel(&mpu, &accel) == ESP_OK) {
            printf("Accel X=%+.3f g | Y=%+.3f g | Z=%+.3f g\n",
                   accel.x, accel.y, accel.z);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
}
