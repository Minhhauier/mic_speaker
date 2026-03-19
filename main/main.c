#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include <math.h>

static const char *TAG = "MAIN";

/* ─── Pin Configuration ──────────────────────────────────────── */
/* Thay đổi theo sơ đồ kết nối thực tế của bạn                   */
#define I2C_MASTER_SCL_IO    22   // GPIO SCL
#define I2C_MASTER_SDA_IO    21   // GPIO SDA
#define I2C_MASTER_PORT      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

/* ─── Test Settings ──────────────────────────────────────────── */
#define TEST_SAMPLE_COUNT    50   // Số mẫu đọc trong test liên tục
#define TEST_INTERVAL_MS     100  // Khoảng cách giữa các mẫu

/* ─── Helper: print separator ───────────────────────────────── */
static void print_sep(char c, int len) {
    for (int i = 0; i < len; i++) putchar(c);
    putchar('\n');
}

/* ─── Test 1: Kiểm tra kết nối (WHO_AM_I) ─────────────────── */
static esp_err_t test_connection(const mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 1: Kiểm tra kết nối (WHO_AM_I)");
    print_sep('-', 50);

    uint8_t who = 0;
    esp_err_t ret = mpu6050_who_am_i(mpu, &who);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Không đọc được WHO_AM_I (%s)", esp_err_to_name(ret));
        return ret;
    }
    if (who == 0x68) {
        ESP_LOGI(TAG, "PASS: WHO_AM_I = 0x%02X (MPU6050 detected)", who);
    } else {
        ESP_LOGW(TAG, "WARN: WHO_AM_I = 0x%02X (unexpected value)", who);
    }
    return ESP_OK;
}

/* ─── Test 2: Self-test ────────────────────────────────────── */
static esp_err_t test_self_test(mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 2: Self-Test phần cứng");
    print_sep('-', 50);

    esp_err_t ret = mpu6050_self_test(mpu);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PASS: Self-test hoàn thành");
    } else {
        ESP_LOGE(TAG, "FAIL: Self-test thất bại");
    }
    return ret;
}

/* ─── Test 3: Đọc nhiệt độ ─────────────────────────────────── */
static esp_err_t test_temperature(const mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 3: Đọc nhiệt độ");
    print_sep('-', 50);

    float temp = 0;
    esp_err_t ret = mpu6050_read_temp(mpu, &temp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Đọc nhiệt độ lỗi (%s)", esp_err_to_name(ret));
        return ret;
    }

    if (temp >= 15.0f && temp <= 85.0f) {
        ESP_LOGI(TAG, "PASS: Nhiệt độ = %.2f °C (trong dải hợp lệ)", temp);
    } else {
        ESP_LOGW(TAG, "WARN: Nhiệt độ = %.2f °C (ngoài dải 15–85°C)", temp);
    }
    return ESP_OK;
}

/* ─── Test 4: Đọc Accel (kiểm tra trọng lực) ──────────────── */
static esp_err_t test_accelerometer(const mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 4: Gia tốc kế (sensor đặt nằm ngang)");
    print_sep('-', 50);

    mpu6050_data_t accel;
    esp_err_t ret = mpu6050_read_accel(mpu, &accel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Đọc accel lỗi (%s)", esp_err_to_name(ret));
        return ret;
    }

    float total_g = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    ESP_LOGI(TAG, "Accel  X: %+7.4f g | Y: %+7.4f g | Z: %+7.4f g", accel.x, accel.y, accel.z);
    ESP_LOGI(TAG, "Độ lớn: %.4f g (lý tưởng ≈ 1.000 g)", total_g);

    float err = fabsf(total_g - 1.0f);
    if (err < 0.15f) {
        ESP_LOGI(TAG, "PASS: Độ lệch %.4f g < 0.15 g", err);
    } else {
        ESP_LOGW(TAG, "WARN: Độ lệch %.4f g >= 0.15 g", err);
    }
    return ESP_OK;
}

/* ─── Test 5: Đọc Gyro (sensor tĩnh) ──────────────────────── */
static esp_err_t test_gyroscope(const mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 5: Con quay hồi chuyển (sensor tĩnh)");
    print_sep('-', 50);

    mpu6050_data_t gyro;
    esp_err_t ret = mpu6050_read_gyro(mpu, &gyro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Đọc gyro lỗi (%s)", esp_err_to_name(ret));
        return ret;
    }

    float drift = fabsf(gyro.x) + fabsf(gyro.y) + fabsf(gyro.z);
    ESP_LOGI(TAG, "Gyro   X: %+8.3f °/s | Y: %+8.3f °/s | Z: %+8.3f °/s",
             gyro.x, gyro.y, gyro.z);
    ESP_LOGI(TAG, "Tổng offset: %.3f °/s (lý tưởng ≈ 0)", drift);

    if (drift < 5.0f) {
        ESP_LOGI(TAG, "PASS: Offset %.3f < 5 °/s", drift);
    } else {
        ESP_LOGW(TAG, "WARN: Offset %.3f >= 5 °/s (có thể cần calibrate)", drift);
    }
    return ESP_OK;
}

/* ─── Test 6: Burst read liên tục ─────────────────────────── */
static esp_err_t test_continuous_burst(const mpu6050_handle_t *mpu)
{
    print_sep('=', 50);
    ESP_LOGI(TAG, "TEST 6: Burst read liên tục (%d mẫu, %d ms/mẫu)",
             TEST_SAMPLE_COUNT, TEST_INTERVAL_MS);
    print_sep('-', 50);

    int success = 0;
    for (int i = 0; i < TEST_SAMPLE_COUNT; i++) {
        mpu6050_data_t accel, gyro;
        float temp;
        esp_err_t ret = mpu6050_read_all(mpu, &accel, &gyro, &temp);
        if (ret == ESP_OK) {
            success++;
            if (i % 10 == 0) {
                ESP_LOGI(TAG, "[%2d/%d] Ax=%+.3f Ay=%+.3f Az=%+.3f g | "
                              "Gx=%+6.2f Gy=%+6.2f Gz=%+6.2f °/s | T=%.1f°C",
                         i + 1, TEST_SAMPLE_COUNT,
                         accel.x, accel.y, accel.z,
                         gyro.x,  gyro.y,  gyro.z, temp);
            }
        } else {
            ESP_LOGE(TAG, "[%2d] Đọc thất bại: %s", i + 1, esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(TEST_INTERVAL_MS));
    }

    int fail = TEST_SAMPLE_COUNT - success;
    ESP_LOGI(TAG, "Kết quả: %d/%d thành công | %d thất bại",
             success, TEST_SAMPLE_COUNT, fail);

    if (fail == 0) {
        ESP_LOGI(TAG, "PASS: Tất cả mẫu đọc OK");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "WARN: %d mẫu lỗi", fail);
        return (fail > TEST_SAMPLE_COUNT / 4) ? ESP_FAIL : ESP_OK;
    }
}

/* ─── Main Task ─────────────────────────────────────────────── */
void app_main(void)
{
    print_sep('*', 50);
    ESP_LOGI(TAG, "  MPU6050 Test Suite — ESP-IDF");
    ESP_LOGI(TAG, "  SCL=GPIO%d  SDA=GPIO%d  400kHz",
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    print_sep('*', 50);
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
    mpu6050_handle_t mpu;

    esp_err_t ret = mpu6050_init(bus_handle, &mpu_cfg, &mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Khởi tạo MPU6050 thất bại: %s", esp_err_to_name(ret));
        // ESP_LOGE(TAG, "Kiểm tra:");
        // ESP_LOGE(TAG, "  1. Kết nối VCC (3.3V), GND, SDA (GPIO%d), SCL (GPIO%d)",
        //          I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        // ESP_LOGE(TAG, "  2. Điện trở pull-up 4.7kΩ trên SDA/SCL (nếu không dùng internal)");
        // ESP_LOGE(TAG, "  3. AD0 = GND → địa chỉ 0x68 | AD0 = VCC → 0x69");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* ── Chạy các Test ── */
    int passed = 0, total = 0;

    #define RUN_TEST(fn, ...) do { \
        total++; \
        if ((fn)(__VA_ARGS__) == ESP_OK) passed++; \
        vTaskDelay(pdMS_TO_TICKS(200)); \
    } while(0)

    // RUN_TEST(test_connection,    &mpu);
    // RUN_TEST(test_self_test,     &mpu);
    // RUN_TEST(test_temperature,   &mpu);
    // RUN_TEST(test_accelerometer, &mpu);
    // RUN_TEST(test_gyroscope,     &mpu);
    // RUN_TEST(test_continuous_burst, &mpu);
    while (1) {
        mpu6050_data_t accel, gyro;
        // float temp;
        // if (mpu6050_read_all(&mpu, &accel, &gyro, &temp) == ESP_OK) {
        //     printf("Ax=%+.3f Ay=%+.3f Az=%+.3f | "
        //            "Gx=%+6.2f Gy=%+6.2f Gz=%+6.2f | T=%.1f°C\n",
        //            accel.x, accel.y, accel.z,
        //            gyro.x,  gyro.y,  gyro.z, temp);
        // }
        if (mpu6050_read_accel(&mpu, &accel) == ESP_OK) {
            printf("Accel X=%+.3f g | Y=%+.3f g | Z=%+.3f g\n",
                   accel.x, accel.y, accel.z);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}