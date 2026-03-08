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
#include <stdint.h>
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
#include "esp_system.h"

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
#define I2S_DMA_DESC_NUM 12
#define I2S_DMA_FRAME_NUM 512
#define I2S_WRITE_CHUNK_BYTES 256
#define I2S_WRITE_TIMEOUT_MS 1000
#define I2S_TIMEOUT_RESET_THRESHOLD 20
#define I2S_TIMEOUT_WARN_THRESHOLD 5
#define AUDIO_FADE_IN_SAMPLES 480
#define AUDIO_TAIL_RAMP_SAMPLES 480
#define DC_BLOCK_ALPHA_Q15 32440
// 0..100 (%): giảm âm lượng loa bằng phần mềm trước khi ghi ra I2S.
#define AUDIO_VOLUME_PERCENT 20

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
static void i2s_write_silence_ms(uint32_t ms);

static esp_err_t i2s_write_all_with_recover(const void *buf, size_t bytes)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t left = bytes;
    int timeout_streak = 0;
    int timeout_warn_count = 0;

    while (left > 0) {
        size_t chunk = left > I2S_WRITE_CHUNK_BYTES ? I2S_WRITE_CHUNK_BYTES : left;
        size_t written = 0;
        esp_err_t err = i2s_channel_write(tx_channel, p, chunk, &written,
                                          pdMS_TO_TICKS(I2S_WRITE_TIMEOUT_MS));
        if (err == ESP_OK && written > 0) {
            p += written;
            left -= written;
            timeout_streak = 0;
            timeout_warn_count = 0;
            continue;
        }

        if (err == ESP_ERR_TIMEOUT) {
            // Timeout nhưng vẫn ghi được dữ liệu -> xem như có tiến triển, không cảnh báo.
            if (written > 0) {
                p += written;
                left -= written;
                timeout_streak = 0;
                timeout_warn_count = 0;
                continue;
            }

            timeout_streak++;
            timeout_warn_count++;
            if (timeout_warn_count >= I2S_TIMEOUT_WARN_THRESHOLD) {
                ESP_LOGW(TAG, "I2S timeout lặp: streak=%d, left=%u",
                         timeout_streak, (unsigned)left);
                timeout_warn_count = 0;
            }
            // Tránh reset channel liên tục vì sẽ tạo tiếng bụp.
            vTaskDelay(pdMS_TO_TICKS(2));
            if (timeout_streak >= I2S_TIMEOUT_RESET_THRESHOLD) {
                ESP_LOGW(TAG, "I2S bị nghẽn lâu, reset channel 1 lần để hồi phục");
                i2s_channel_disable(tx_channel);
                i2s_channel_enable(tx_channel);
                timeout_streak = 0;
            }
            continue;
        }

        ESP_LOGW(TAG, "I2S ghi thất bại: err=%s, written=%u, left=%u",
                 esp_err_to_name(err), (unsigned)written, (unsigned)left);
        return err == ESP_OK ? ESP_FAIL : err;
    }

    return ESP_OK;
}

static void i2s_write_silence_ms(uint32_t ms)
{
    int16_t zero[512] = {0};
    uint32_t frames = (SAMPLE_RATE * ms) / 1000;
    uint32_t samples_total = frames * 2; // stereo
    size_t bytes_left = (size_t)samples_total * sizeof(int16_t);

    while (bytes_left > 0) {
        size_t bytes = bytes_left > sizeof(zero) ? sizeof(zero) : bytes_left;
        (void)i2s_write_all_with_recover(zero, bytes);
        bytes_left -= bytes;
    }
}

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
    // Giảm nhiễu xung do modem sleep khi đang phát audio.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

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
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
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

    // Đệm im lặng ngắn trước khi phát để giảm pop đầu câu.
    i2s_write_silence_ms(20);

    // minimp3 trả về số sample PCM (interleaved nếu stereo), tối đa 2304 sample/frame.
    static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
    mp3dec_frame_info_t info;

    const uint8_t *ptr       = data;
    size_t         remaining = len;
    int            frame_count = 0;
    size_t         played_samples = 0;
    int16_t        tail_l = 0;
    int16_t        tail_r = 0;
    int32_t        dc_x_l = 0, dc_y_l = 0;
    int32_t        dc_x_r = 0, dc_y_r = 0;

    while (remaining > 0) {
        memset(&info, 0, sizeof(info));
        int samples = mp3dec_decode_frame(dec, ptr, (int)remaining, pcm, &info);

        if (info.frame_bytes == 0) {
            ptr++;
            remaining--;
            continue;
        }

        if ((size_t)info.frame_bytes > remaining) {
            ESP_LOGW(TAG, "Frame lỗi: frame_bytes=%d > remaining=%u", info.frame_bytes, (unsigned)remaining);
            break;
        }

        if (samples > 0) {
            if (info.channels != 1 && info.channels != 2) {
                ESP_LOGW(TAG, "Skip frame: channels=%d", info.channels);
                ptr       += info.frame_bytes;
                remaining -= info.frame_bytes;
                continue;
            }

            if (samples > MINIMP3_MAX_SAMPLES_PER_FRAME) {
                ESP_LOGW(TAG, "Skip frame: samples=%d vượt MAX=%d", samples, MINIMP3_MAX_SAMPLES_PER_FRAME);
                ptr       += info.frame_bytes;
                remaining -= info.frame_bytes;
                continue;
            }

            // Tránh đổi clock I2S giữa chừng vì dễ tạo tiếng bụp.
            if (info.hz > 0 && (uint32_t)info.hz != SAMPLE_RATE) {
                ESP_LOGW(TAG, "MP3 rate=%d khác SAMPLE_RATE=%d (bỏ qua reconfig để tránh pop)",
                         info.hz, SAMPLE_RATE);
            }

            size_t out_samples;
            if (info.channels == 1) {
                // Mono → Stereo (từ cuối về đầu tránh ghi đè)
                for (int i = samples - 1; i >= 0; i--) {
                    pcm[i * 2 + 1] = pcm[i];
                    pcm[i * 2]     = pcm[i];
                }
                out_samples = (size_t)samples * 2;
            } else {
                // minimp3 trả về số sample mỗi kênh, nên cần nhân số kênh.
                out_samples = (size_t)samples * (size_t)info.channels;
            }

            if (out_samples > (size_t)MINIMP3_MAX_SAMPLES_PER_FRAME) {
                ESP_LOGW(TAG, "Skip frame: out_samples=%u vượt MAX=%d", (unsigned)out_samples, MINIMP3_MAX_SAMPLES_PER_FRAME);
                ptr       += info.frame_bytes;
                remaining -= info.frame_bytes;
                continue;
            }

            // Scale volume theo phần trăm để tránh loa quá to/gây sụt áp.
            if (AUDIO_VOLUME_PERCENT < 100) {
                for (size_t i = 0; i < out_samples; i++) {
                    int32_t scaled = ((int32_t)pcm[i] * AUDIO_VOLUME_PERCENT) / 100;
                    if (scaled > INT16_MAX) scaled = INT16_MAX;
                    if (scaled < INT16_MIN) scaled = INT16_MIN;
                    pcm[i] = (int16_t)scaled;
                }
            }

            // Fade-in mềm ở đầu câu để giảm click/pop.
            if (played_samples < AUDIO_FADE_IN_SAMPLES) {
                for (size_t i = 0; i < out_samples && played_samples < AUDIO_FADE_IN_SAMPLES; i++, played_samples++) {
                    int32_t s = pcm[i];
                    s = (s * (int32_t)played_samples) / (int32_t)AUDIO_FADE_IN_SAMPLES;
                    pcm[i] = (int16_t)s;
                }
            }

            // DC-block IIR để giảm tiếng bụp/click thấp tần.
            for (size_t i = 0; i + 1 < out_samples; i += 2) {
                int32_t x_l = pcm[i];
                int32_t y_l = x_l - dc_x_l + (int32_t)(((int64_t)dc_y_l * DC_BLOCK_ALPHA_Q15) >> 15);
                dc_x_l = x_l;
                dc_y_l = y_l;
                if (y_l > INT16_MAX) y_l = INT16_MAX;
                if (y_l < INT16_MIN) y_l = INT16_MIN;
                pcm[i] = (int16_t)y_l;

                int32_t x_r = pcm[i + 1];
                int32_t y_r = x_r - dc_x_r + (int32_t)(((int64_t)dc_y_r * DC_BLOCK_ALPHA_Q15) >> 15);
                dc_x_r = x_r;
                dc_y_r = y_r;
                if (y_r > INT16_MAX) y_r = INT16_MAX;
                if (y_r < INT16_MIN) y_r = INT16_MIN;
                pcm[i + 1] = (int16_t)y_r;
            }

            if (out_samples >= 2) {
                tail_l = pcm[out_samples - 2];
                tail_r = pcm[out_samples - 1];
            }

            size_t pcm_bytes = out_samples * sizeof(int16_t);
            esp_err_t wr_err = i2s_write_all_with_recover(pcm, pcm_bytes);
            if (wr_err != ESP_OK) {
                ESP_LOGW(TAG, "Bỏ qua frame do lỗi I2S: err=%s", esp_err_to_name(wr_err));
            } else {
                frame_count++;
            }
        }

        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
    }

    free(dec);

    // Ramp biên độ về 0 ở cuối câu để giảm pop ở điểm dừng.
    {
        int16_t tail[AUDIO_TAIL_RAMP_SAMPLES];
        size_t n = AUDIO_TAIL_RAMP_SAMPLES;
        if (n & 1U) n--; // đảm bảo số sample chẵn (stereo interleaved)
        for (size_t i = 0; i < n; i += 2) {
            int32_t remain = (int32_t)(n - i);
            tail[i]     = (int16_t)(((int32_t)tail_l * remain) / (int32_t)n);
            tail[i + 1] = (int16_t)(((int32_t)tail_r * remain) / (int32_t)n);
        }
        (void)i2s_write_all_with_recover(tail, n * sizeof(int16_t));
    }

    // Đệm im lặng ngắn sau khi phát để giảm pop cuối câu.
    i2s_write_silence_ms(25);
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
    // brownout_hal_config_t cfg = {
    //     .threshold = 0,
    //     .enabled   = false,
    // };
    // brownout_hal_config(&cfg);
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGW(TAG, "Reset reason: %d", (int)reason);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== ESP32 Tiếng Việt TTS (ESP-IDF v5.x) ===");
    ESP_LOGI(TAG, "Heap khả dụng: %d bytes", (int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Volume: %d%%", AUDIO_VOLUME_PERCENT);

    wifi_init();
    i2s_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(test_audio_task, "test_audio_task", 1024*16, NULL,5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}