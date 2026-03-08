// ============================================================
// ESP32 + MAX98357A | ESP-IDF v5.x | Google TTS tiếng Việt
// BCLK=GPIO27, LRC=GPIO14, DOUT=GPIO13
// Fix: forward declaration, minimp3 đúng chỗ, NULL check
// ============================================================

// ── minimp3 PHẢI define trước mọi include khác ───────────────
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3.h"   // đặt file minimp3.h trong cùng thư mục main/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"

#include "esp_private/esp_clk.h"
#include "hal/brownout_hal.h"
// ─── Cấu hình WiFi ───────────────────────────────────────────
#define WIFI_SSID       "ABC"
#define WIFI_PASS       "coinhe2018"
#define WIFI_MAX_RETRY  5

// ─── Chân I2S ────────────────────────────────────────────────
#define I2S_BCLK_PIN    GPIO_NUM_27
#define I2S_LRC_PIN     GPIO_NUM_14
#define I2S_DOUT_PIN    GPIO_NUM_13

// ─── Cấu hình audio ──────────────────────────────────────────
#define SAMPLE_RATE     24000
#define I2S_PORT        I2S_NUM_0
#define HTTP_BUF_SIZE   (4 * 1024)
#define MP3_BUF_SIZE    (64 * 1024)

static const char *TAG = "VN_TTS";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retry_count = 0;

static i2s_chan_handle_t tx_channel = NULL;

// Buffer MP3 tải về
static uint8_t *mp3_data = NULL;
static size_t   mp3_len  = 0;
static size_t   mp3_cap  = 0;

// ─── Forward declarations ─────────────────────────────────────
static void play_mp3_data(const uint8_t *data, size_t len);
static void url_encode(const char *input, char *output, size_t out_size);

// =============================================================
// WiFi
// =============================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Thử lại WiFi %d/%d...", retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi OK! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Kết nối WiFi thành công!");
    } else {
        ESP_LOGE(TAG, "Kết nối WiFi thất bại!");
    }
}

// =============================================================
// I2S
// =============================================================
static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_channel, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_LRC_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_channel, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_channel));
    ESP_LOGI(TAG, "I2S OK (BCLK=%d, LRC=%d, DOUT=%d)",
             I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DOUT_PIN);
}

// =============================================================
// HTTP – gom dữ liệu MP3
// =============================================================
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (!evt->data || evt->data_len <= 0) return ESP_OK;

        if (mp3_len + (size_t)evt->data_len > mp3_cap) {
            size_t new_cap = mp3_cap + HTTP_BUF_SIZE * 8;
            uint8_t *tmp = realloc(mp3_data, new_cap);
            if (!tmp) {
                ESP_LOGE(TAG, "Hết RAM! (cần %d bytes)", (int)new_cap);
                return ESP_FAIL;
            }
            mp3_data = tmp;
            mp3_cap  = new_cap;
        }
        memcpy(mp3_data + mp3_len, evt->data, evt->data_len);
        mp3_len += evt->data_len;
    } else if (evt->event_id == HTTP_EVENT_ERROR) {
        ESP_LOGE(TAG, "HTTP lỗi!");
    }
    return ESP_OK;
}

// =============================================================
// URL encode UTF-8
// =============================================================
static void url_encode(const char *input, char *output, size_t out_size)
{
    size_t j = 0;
    for (size_t i = 0; input[i] != '\0' && j + 4 < out_size; i++) {
        unsigned char c = (unsigned char)input[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            output[j++] = (char)c;
        } else if (c == ' ') {
            output[j++] = '+';
        } else {
            j += snprintf(output + j, out_size - j, "%%%02X", c);
        }
    }
    output[j] = '\0';
}

// =============================================================
// Phát MP3 buffer qua I2S dùng minimp3
// =============================================================
static void play_mp3_data(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        ESP_LOGE(TAG, "Buffer MP3 rỗng!");
        return;
    }

    // Cấp phát decoder trên HEAP – tránh stack overflow
    mp3dec_t *dec = malloc(sizeof(mp3dec_t));
    if (!dec) {
        ESP_LOGE(TAG, "Không đủ RAM cho decoder!");
        return;
    }
    mp3dec_init(dec);

    // PCM buffer tĩnh: 1 frame = tối đa 1152 samples × stereo × 2 bytes
    static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME * 2];
    mp3dec_frame_info_t info;

    const uint8_t *ptr       = data;
    size_t         remaining = len;
    int            frame_count = 0;
    bool           rate_set    = false;

    while (remaining > 0) {
        memset(&info, 0, sizeof(info));
        int samples = mp3dec_decode_frame(dec, ptr, (int)remaining, pcm, &info);

        if (info.frame_bytes == 0) {
            ptr++;
            remaining--;
            continue;
        }

        if (samples > 0) {
            // Cập nhật sample rate theo frame đầu tiên
            if (!rate_set) {
                rate_set = true;
                if (info.hz > 0 && (uint32_t)info.hz != SAMPLE_RATE) {
                    ESP_LOGI(TAG, "Điều chỉnh sample rate: %d Hz", info.hz);
                    i2s_channel_disable(tx_channel);
                    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(info.hz);
                    i2s_channel_reconfig_std_clock(tx_channel, &clk);
                    i2s_channel_enable(tx_channel);
                }
            }

            size_t pcm_bytes;
            if (info.channels == 1) {
                // Mono → Stereo (từ cuối về đầu tránh ghi đè)
                for (int i = samples - 1; i >= 0; i--) {
                    pcm[i * 2 + 1] = pcm[i];
                    pcm[i * 2]     = pcm[i];
                }
                pcm_bytes = (size_t)samples * 2 * sizeof(int16_t);
            } else {
                pcm_bytes = (size_t)samples * (size_t)info.channels * sizeof(int16_t);
            }

            size_t written = 0;
            i2s_channel_write(tx_channel, pcm, pcm_bytes, &written, pdMS_TO_TICKS(2000));
            frame_count++;
        }

        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
    }

    free(dec);
    ESP_LOGI(TAG, "Phát xong! (%d frames)", frame_count);
}

// =============================================================
// speak_vietnamese
// =============================================================
static void speak_vietnamese(const char *text)
{
    ESP_LOGI(TAG, "Phát: %s", text);

    mp3_len = 0;
    if (mp3_data == NULL) {
        mp3_cap  = MP3_BUF_SIZE;
        mp3_data = malloc(mp3_cap);
        if (!mp3_data) {
            ESP_LOGE(TAG, "Không cấp được buffer MP3!");
            return;
        }
    }

    char encoded[512] = {0};
    url_encode(text, encoded, sizeof(encoded));

    char url[640] = {0};
    snprintf(url, sizeof(url),
             "http://translate.google.com/translate_tts"
             "?ie=UTF-8&client=tw-ob&tl=vi&q=%s", encoded);

    esp_http_client_config_t config = {
        .url           = url,
        .event_handler = http_event_handler,
        .buffer_size   = HTTP_BUF_SIZE,
        .timeout_ms    = 15000,
        .user_agent    = "Mozilla/5.0",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Không tạo được HTTP client!");
        return;
    }

    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP %d | MP3: %d bytes", status, (int)mp3_len);

    if (err == ESP_OK && status == 200 && mp3_len > 0) {
        play_mp3_data(mp3_data, mp3_len);
    } else {
        ESP_LOGE(TAG, "Lỗi TTS: %s (HTTP %d)", esp_err_to_name(err), status);
    }

    esp_http_client_cleanup(client);
    vTaskDelay(pdMS_TO_TICKS(300));
}

// =============================================================
// app_main
// =============================================================
void test_audio_task(void *arg){
    speak_vietnamese("Xin chào!");
    vTaskDelay(pdMS_TO_TICKS(800));

    speak_vietnamese("Hệ thống đã sẵn sàng.");
    vTaskDelay(pdMS_TO_TICKS(800));

    speak_vietnamese("Chào mừng bạn ");
    vTaskDelay(pdMS_TO_TICKS(800));

    speak_vietnamese("Cảm ơn bạn đã sử dụng.");

    free(mp3_data);
    mp3_data = NULL;

    ESP_LOGI(TAG, "Hoàn tất!");
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}
void app_main(void)
{
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    //     // Cách đúng cho ESP-IDF v5.x
    brownout_hal_config_t cfg = {
        .threshold = 0,
        .enabled   = false,
    };
    brownout_hal_config(&cfg);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== ESP32 Tiếng Việt TTS (ESP-IDF v5.x) ===");
    ESP_LOGI(TAG, "Heap khả dụng: %d bytes", (int)esp_get_free_heap_size());

    wifi_init();
    i2s_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(test_audio_task, "test_audio_task", 1024*16, NULL,5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}