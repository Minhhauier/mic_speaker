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
#include <stdbool.h>
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
#define SAMPLE_RATE     22050
#define I2S_PORT        I2S_NUM_0
#define HTTP_BUF_SIZE   (4 * 1024)
#define MP3_BUF_SIZE    (64 * 1024)
#define I2S_DMA_DESC_NUM 8
#define I2S_DMA_FRAME_NUM 256
#define I2S_WRITE_CHUNK_BYTES 2048
#define AUDIO_PRE_SILENCE_SAMPLES 512
#define AUDIO_POST_SILENCE_SAMPLES 1024
#define AUDIO_FADE_IN_SAMPLES 480
#define AUDIO_FADE_OUT_SAMPLES 480
#define DC_BLOCK_ALPHA_Q15 32440
#define TTS_CACHE_NAMESPACE "tts_cache"
#define TTS_CACHE_MAX_ENTRY_BYTES (24 * 1024)
#define ENABLE_TTS_CACHE 0
// 0..100 (%): giảm âm lượng loa bằng phần mềm trước khi ghi ra I2S.
#define AUDIO_VOLUME_PERCENT 20

static const char *TAG = "VN_TTS";

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int retry_count = 0;

static i2s_chan_handle_t tx_channel = NULL;
static mp3dec_t s_mp3_decoder;

// Buffer MP3 tải về
static uint8_t *mp3_data = NULL;
static size_t   mp3_len  = 0;
static size_t   mp3_cap  = 0;
static bool     tts_cache_enabled = true;

// ─── Forward declarations ─────────────────────────────────────
static void play_mp3_data(const uint8_t *data, size_t len);
static void url_encode(const char *input, char *output, size_t out_size);
static bool ensure_mp3_capacity(size_t needed);
static uint32_t fnv1a_32(const char *s);
static bool tts_cache_load(const char *text);
static void tts_cache_store(const char *text, const uint8_t *data, size_t len);
static void i2s_write_silence_samples(size_t samples);

static void i2s_write_silence_samples(size_t samples)
{
    int16_t zeros[256] = {0};
    size_t left = samples;
    while (left > 0) {
        size_t n = left > 256 ? 256 : left;
        size_t written = 0;
        (void)i2s_channel_write(tx_channel, zeros, n * sizeof(int16_t), &written, portMAX_DELAY);
        left -= n;
    }
}

static uint32_t current_i2s_rate = SAMPLE_RATE;

static void i2s_set_sample_rate(uint32_t rate) {
    if (rate == current_i2s_rate || rate == 0) return;
    
    i2s_channel_disable(tx_channel);
    
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    i2s_channel_reconfig_std_clock(tx_channel, &clk_cfg);
    
    i2s_channel_enable(tx_channel);
    current_i2s_rate = rate;
    
    ESP_LOGI(TAG, "I2S rate → %d Hz", (int)rate);
}

static bool ensure_mp3_capacity(size_t needed)
{
    if (needed == 0) return false;
    if (mp3_data != NULL && mp3_cap >= needed) return true;

    uint8_t *tmp = realloc(mp3_data, needed);
    if (!tmp) return false;

    mp3_data = tmp;
    mp3_cap = needed;
    return true;
}

static uint32_t fnv1a_32(const char *s)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; s[i] != '\0'; i++) {
        h ^= (uint8_t)s[i];
        h *= 16777619u;
    }
    return h;
}

static bool tts_cache_load(const char *text)
{
    if (!ENABLE_TTS_CACHE) {
        return false;
    }

    if (!tts_cache_enabled) {
        return false;
    }

    nvs_handle_t h;
    if (nvs_open(TTS_CACHE_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }

    uint32_t hash = fnv1a_32(text);
    char key_len[16] = {0};
    char key_data[16] = {0};
    snprintf(key_len, sizeof(key_len), "l_%08lx", (unsigned long)hash);
    snprintf(key_data, sizeof(key_data), "d_%08lx", (unsigned long)hash);

    uint32_t len32 = 0;
    esp_err_t err = nvs_get_u32(h, key_len, &len32);
    if (err != ESP_OK || len32 == 0 || len32 > TTS_CACHE_MAX_ENTRY_BYTES) {
        nvs_close(h);
        return false;
    }

    if (!ensure_mp3_capacity((size_t)len32)) {
        nvs_close(h);
        ESP_LOGE(TAG, "Không đủ RAM để đọc cache (%u bytes)", (unsigned)len32);
        return false;
    }

    size_t blob_len = (size_t)len32;
    err = nvs_get_blob(h, key_data, mp3_data, &blob_len);
    nvs_close(h);
    if (err != ESP_OK || blob_len != (size_t)len32) {
        return false;
    }

    mp3_len = blob_len;
    ESP_LOGI(TAG, "Cache hit: %u bytes", (unsigned)mp3_len);
    return true;
}

static void tts_cache_store(const char *text, const uint8_t *data, size_t len)
{
    if (!ENABLE_TTS_CACHE) {
        return;
    }

    if (!tts_cache_enabled) {
        return;
    }

    if (!data || len == 0 || len > TTS_CACHE_MAX_ENTRY_BYTES) {
        return;
    }

    nvs_handle_t h;
    if (nvs_open(TTS_CACHE_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    uint32_t hash = fnv1a_32(text);
    char key_len[16] = {0};
    char key_data[16] = {0};
    snprintf(key_len, sizeof(key_len), "l_%08lx", (unsigned long)hash);
    snprintf(key_data, sizeof(key_data), "d_%08lx", (unsigned long)hash);

    esp_err_t err = nvs_set_blob(h, key_data, data, len);
    if (err == ESP_OK) {
        err = nvs_set_u32(h, key_len, (uint32_t)len);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Cache store OK: %u bytes", (unsigned)len);
    } else {
        ESP_LOGW(TAG, "Cache store fail: %s", esp_err_to_name(err));
        if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE) {
            tts_cache_enabled = false;
            ESP_LOGW(TAG, "NVS đầy, tắt cache NVS để tránh lỗi lặp.");
        }
    }
}

static esp_err_t i2s_write_all_with_recover(const void *buf, size_t bytes)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t left = bytes;

    while (left > 0) {
        size_t chunk = left > I2S_WRITE_CHUNK_BYTES ? I2S_WRITE_CHUNK_BYTES : left;
        size_t written = 0;
        esp_err_t err = i2s_channel_write(tx_channel, p, chunk, &written, portMAX_DELAY);
        if (err == ESP_OK && written > 0) {
            p += written;
            left -= written;
            continue;
        }

        ESP_LOGW(TAG, "I2S ghi thất bại: err=%s, written=%u, left=%u",
                 esp_err_to_name(err), (unsigned)written, (unsigned)left);
        return err == ESP_OK ? ESP_FAIL : err;
    }

    return ESP_OK;
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
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
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
            size_t needed = mp3_len + (size_t)evt->data_len;
            size_t new_cap = (mp3_cap > 0) ? mp3_cap : (HTTP_BUF_SIZE * 2);
            while (new_cap < needed) {
                size_t next = new_cap + HTTP_BUF_SIZE;
                if (next <= new_cap) {
                    ESP_LOGE(TAG, "Tràn kích thước buffer MP3!");
                    return ESP_FAIL;
                }
                new_cap = next;
            }

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

    mp3dec_t *dec = &s_mp3_decoder;
    mp3dec_init(dec);

    // Đệm im lặng ngắn trước câu để tránh pop ở điểm bắt đầu.
    i2s_write_silence_samples(AUDIO_PRE_SILENCE_SAMPLES);

    // Tạm tắt silence đầu câu để tránh chèn thêm write khi TX đang nghẽn.

    // minimp3 trả về số sample PCM (interleaved nếu stereo), tối đa 2304 sample/frame.
    static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
    mp3dec_frame_info_t info;

    const uint8_t *ptr       = data;
    size_t         remaining = len;
    int            frame_count = 0;
    int            no_sync_count = 0;
    size_t         played_samples = 0;
    int32_t        dc_x_l = 0, dc_y_l = 0;
    int32_t        dc_x_r = 0, dc_y_r = 0;
    
    while (remaining > 0) {
        memset(&info, 0, sizeof(info));
        int samples = mp3dec_decode_frame(dec, ptr, (int)remaining, pcm, &info);

        if (info.frame_bytes == 0) {
            ptr++;
            remaining--;
            no_sync_count++;
            // if ((no_sync_count & 0x3FF) == 0) {
            //     taskYIELD();
            // }
            continue;
        }

        no_sync_count = 0;

        if ((size_t)info.frame_bytes > remaining) {
            ESP_LOGW(TAG, "Frame lỗi: frame_bytes=%d > remaining=%u", info.frame_bytes, (unsigned)remaining);
            break;
        }
        if (info.hz > 0) {
           i2s_set_sample_rate((uint32_t)info.hz);
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

            // // Tránh đổi clock I2S giữa chừng vì dễ tạo tiếng bụp.
            // if (info.hz > 0 && (uint32_t)info.hz != SAMPLE_RATE) {
            //     ESP_LOGW(TAG, "MP3 rate=%d khác SAMPLE_RATE=%d (bỏ qua reconfig để tránh pop)",
            //              info.hz, SAMPLE_RATE);
            // }

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

            const bool is_last_frame = ((size_t)info.frame_bytes == remaining);

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
                    s = (s * (int32_t)played_samples+1) / (int32_t)AUDIO_FADE_IN_SAMPLES;
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

            // Fade-out mềm ở frame cuối để hạn chế click/pop khi kết thúc.
            if (is_last_frame && AUDIO_FADE_OUT_SAMPLES > 0) {
                size_t fade = out_samples < AUDIO_FADE_OUT_SAMPLES ? out_samples : AUDIO_FADE_OUT_SAMPLES;
                size_t start = out_samples - fade;
                for (size_t i = 0; i < fade; i++) {
                    int32_t gain_num = (int32_t)(fade - i);
                    int32_t s = pcm[start + i];
                    s = (s * gain_num) / (int32_t)fade;
                    pcm[start + i] = (int16_t)s;
                }
            }

            size_t pcm_bytes = out_samples * sizeof(int16_t);
            esp_err_t wr_err = i2s_write_all_with_recover(pcm, pcm_bytes);
          //  printf("okee\n");
            if (wr_err != ESP_OK) {
                ESP_LOGW(TAG, "Bỏ qua frame do lỗi I2S: err=%s", esp_err_to_name(wr_err));
            } else {
                frame_count++;
            }

            if ((frame_count & 0x0F) == 0) {
                // Nhường CPU định kỳ để tránh Task WDT khi phát file dài.
                taskYIELD();
            }
        }

        ptr       += info.frame_bytes;
        remaining -= info.frame_bytes;
    }

    // Đệm im lặng ngắn sau câu để tránh pop ở điểm kết thúc.
    i2s_write_silence_samples(AUDIO_POST_SILENCE_SAMPLES);

    // Tạm tắt tail ramp/silence cuối câu để tránh kẹt TX.
    ESP_LOGI(TAG, "Phát xong! (%d frames)", frame_count);
}

// =============================================================
// speak_vietnamese
// =============================================================
static void speak_vietnamese(const char *text)
{
    ESP_LOGI(TAG, "Phát: %s", text);
    ESP_LOGI(TAG,
             "Heap trước HTTP: free=%u, largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    mp3_len = 0;

    // Cùng câu đã từng gặp -> phát từ cache, không cần gọi mạng.
    if (tts_cache_load(text)) {
        play_mp3_data(mp3_data, mp3_len);
        vTaskDelay(pdMS_TO_TICKS(300));
        return;
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
        tts_cache_store(text, mp3_data, mp3_len);
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
    speak_vietnamese("Xin chào! hệ thống đã sẵn sàng rất vui được gặp bạn, tôi do master Minh chế tạo.");
    vTaskDelay(pdMS_TO_TICKS(800));

    // speak_vietnamese("Hệ thống đã sẵn sàng.");
    // vTaskDelay(pdMS_TO_TICKS(800));

    // speak_vietnamese("Chào mừng bạn ");
    // vTaskDelay(pdMS_TO_TICKS(800));

    // speak_vietnamese("Cảm ơn bạn đã sử dụng.");

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

    // Tạm thời bỏ pipeline streambuffer/task TX để ưu tiên ổn định, tránh reboot.
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(test_audio_task, "test_audio_task", 1024*24, NULL,5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}