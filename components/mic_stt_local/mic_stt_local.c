#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"


#include "speaker.h"
// ================= CONFIG =================
#define WS_URL           "ws://10.183.3.28:8000/ws/transcribe"

#define SAMPLE_RATE      16000
#define VAD_CHUNK        512
#define VAD_THRESHOLD    800
#define VAD_SILENCE_MS   1500
#define VAD_SILENCE_CHUNKS (VAD_SILENCE_MS * SAMPLE_RATE / 1000 / VAD_CHUNK)
#define VAD_PRE_ROLL     20
#define MAX_RECORD_SEC   10
#define MAX_SAMPLES      (SAMPLE_RATE * MAX_RECORD_SEC)

#define SEND_CHUNK       4096
#define RESP_BUF_SIZE    2048

// Event bits
#define WS_CONNECTED_BIT   BIT0
#define WS_RESPONSE_BIT    BIT1

static const char *TAG = "mic_stt";

// ================= GLOBALS =================
static i2s_chan_handle_t   rx_handle   = NULL;
static esp_websocket_client_handle_t ws_client = NULL;
static EventGroupHandle_t  ws_evt_grp  = NULL;
static int16_t            *audio_buf   = NULL;
static char                response_buf[RESP_BUF_SIZE];

// ================= I2S INIT =================
static void i2s_mic_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_1, I2S_ROLE_MASTER
    );
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk  = I2S_GPIO_UNUSED,
            .bclk  = GPIO_NUM_33,
            .ws    = GPIO_NUM_25,
            .dout  = I2S_GPIO_UNUSED,
            .din   = GPIO_NUM_32,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    ESP_LOGI(TAG, "I2S init done");
}

// ================= WEBSOCKET =================
static void ws_event_handler(void *arg,
                              esp_event_base_t base,
                              int32_t event_id,
                              void *event_data)
{
    esp_websocket_event_data_t *data =
        (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WS connected");
            xEventGroupSetBits(ws_evt_grp, WS_CONNECTED_BIT);
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WS disconnected, reconnecting...");
            xEventGroupClearBits(ws_evt_grp, WS_CONNECTED_BIT);
            break;

        case WEBSOCKET_EVENT_DATA:
            // Chỉ xử lý text frame (op_code 0x01)
            if (data->op_code == 0x01 && data->data_len > 0) {
                int copy = data->data_len < RESP_BUF_SIZE - 1
                           ? data->data_len : RESP_BUF_SIZE - 1;
                memcpy(response_buf, data->data_ptr, copy);
                response_buf[copy] = '\0';
                xEventGroupSetBits(ws_evt_grp, WS_RESPONSE_BIT);
            }
            break;

        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WS error");
            break;

        default:
            break;
    }
}

static void ws_init(void)
{
    ws_evt_grp = xEventGroupCreate();

    esp_websocket_client_config_t ws_cfg = {
        .uri                  = WS_URL,
        .reconnect_timeout_ms = 3000,
        .network_timeout_ms   = 10000,
    };

    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(ws_client, WEBSOCKET_EVENT_ANY,
                                  ws_event_handler, NULL);
    esp_websocket_client_start(ws_client);

    ESP_LOGI(TAG, "Đang kết nối WebSocket...");
    EventBits_t bits = xEventGroupWaitBits(
        ws_evt_grp, WS_CONNECTED_BIT,
        pdFALSE, pdTRUE,
        pdMS_TO_TICKS(10000)
    );

    if (bits & WS_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WebSocket sẵn sàng");
    } else {
        ESP_LOGE(TAG, "Timeout kết nối WebSocket!");
    }
}

// ================= RMS =================
static float calc_rms(int16_t *buf, int len)
{
    float sum = 0;
    for (int i = 0; i < len; i++)
        sum += (float)buf[i] * buf[i];
    return sqrtf(sum / len);
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int utf8_encode(uint32_t cp, char *out)
{
    if (cp <= 0x7F) {
        out[0] = (char)cp;
        return 1;
    }
    if (cp <= 0x7FF) {
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    }
    if (cp <= 0xFFFF) {
        out[0] = (char)(0xE0 | (cp >> 12));
        out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[2] = (char)(0x80 | (cp & 0x3F));
        return 3;
    }
    out[0] = '?';
    return 1;
}

// Decode JSON string escape sequences in-place: \n, \", \\ and \uXXXX.
static void json_unescape_inplace(char *s)
{
    char *src = s;
    char *dst = s;

    while (*src) {
        if (*src != '\\') {
            *dst++ = *src++;
            continue;
        }

        src++;
        if (*src == '\0') {
            break;
        }

        switch (*src) {
            case '"': *dst++ = '"'; src++; break;
            case '\\': *dst++ = '\\'; src++; break;
            case '/': *dst++ = '/'; src++; break;
            case 'b': *dst++ = '\b'; src++; break;
            case 'f': *dst++ = '\f'; src++; break;
            case 'n': *dst++ = '\n'; src++; break;
            case 'r': *dst++ = '\r'; src++; break;
            case 't': *dst++ = '\t'; src++; break;
            case 'u': {
                if (src[1] && src[2] && src[3] && src[4]) {
                    int h1 = hex_val(src[1]);
                    int h2 = hex_val(src[2]);
                    int h3 = hex_val(src[3]);
                    int h4 = hex_val(src[4]);
                    if (h1 >= 0 && h2 >= 0 && h3 >= 0 && h4 >= 0) {
                        uint32_t cp = (uint32_t)((h1 << 12) | (h2 << 8) | (h3 << 4) | h4);
                        char utf8[4] = {0};
                        int n = utf8_encode(cp, utf8);
                        for (int i = 0; i < n; i++) {
                            *dst++ = utf8[i];
                        }
                        src += 5;
                        break;
                    }
                }
                *dst++ = '?';
                src++;
                break;
            }
            default:
                *dst++ = *src++;
                break;
        }
    }

    *dst = '\0';
}

// ================= VAD RECORD =================
static int record_with_vad(void)
{
    static int16_t pre_roll_buf[VAD_PRE_ROLL][VAD_CHUNK];
    int   pre_roll_idx = 0;
    int16_t chunk[VAD_CHUNK];
    int32_t raw;
    size_t  bytes_read;

    bool voice_started = false;
    int  silence_count = 0;
    int  total_samples = 0;

    ESP_LOGI(TAG, "Chờ giọng nói...");

    while (1) {
        // Đọc 1 chunk
        for (int i = 0; i < VAD_CHUNK; i++) {
            esp_err_t ret;
            do {
                ret = i2s_channel_read(rx_handle, &raw, sizeof(raw),
                                       &bytes_read, portMAX_DELAY);
            } while (ret != ESP_OK || bytes_read != sizeof(raw));
            chunk[i] = (int16_t)(raw >> 14);
        }

        float rms = calc_rms(chunk, VAD_CHUNK);

        if (!voice_started) {
            memcpy(pre_roll_buf[pre_roll_idx], chunk,
                   VAD_CHUNK * sizeof(int16_t));
            pre_roll_idx = (pre_roll_idx + 1) % VAD_PRE_ROLL;

            if (rms > VAD_THRESHOLD) {
                ESP_LOGI(TAG, "Phát hiện giọng! RMS=%.0f", rms);
                voice_started = true;

                // Nạp pre-roll vào buffer chính
                for (int k = 0; k < VAD_PRE_ROLL; k++) {
                    int idx = (pre_roll_idx + k) % VAD_PRE_ROLL;
                    if (total_samples + VAD_CHUNK <= MAX_SAMPLES) {
                        memcpy(audio_buf + total_samples,
                               pre_roll_buf[idx],
                               VAD_CHUNK * sizeof(int16_t));
                        total_samples += VAD_CHUNK;
                    }
                }
            }
        } else {
            if (total_samples + VAD_CHUNK <= MAX_SAMPLES) {
                memcpy(audio_buf + total_samples, chunk,
                       VAD_CHUNK * sizeof(int16_t));
                total_samples += VAD_CHUNK;
            } else {
                ESP_LOGW(TAG, "Đạt giới hạn %ds, dừng ghi.", MAX_RECORD_SEC);
                break;
            }

            if (rms < VAD_THRESHOLD) {
                if (++silence_count >= VAD_SILENCE_CHUNKS) {
                    ESP_LOGI(TAG, "Im lặng %.1fs, kết thúc.",
                             VAD_SILENCE_MS / 1000.0f);
                    break;
                }
            } else {
                silence_count = 0;
            }
        }
    }

    ESP_LOGI(TAG, "Ghi xong: %d samples (%.1fs)",
             total_samples, (float)total_samples / SAMPLE_RATE);
    return total_samples;
}

// ================= PARSE & HIỂN THỊ =================
static void handle_stt_result(void)
{
    ESP_LOGI(TAG, "Response: %s", response_buf);

    char *key = strstr(response_buf, "\"text\"");
    if (!key) {
        ESP_LOGW(TAG, "Không tìm thấy text trong response");
        return;
    }

    char *p = strchr(key, ':');
    if (!p) {
        ESP_LOGW(TAG, "Response sai format (không có ':')");
        return;
    }

    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }

    if (*p != '"') {
        ESP_LOGW(TAG, "Response sai format (text không phải chuỗi)");
        return;
    }

    p++;
    char *end = strchr(p, '"');
    if (end) *end = '\0';

    json_unescape_inplace(p);

    ESP_LOGI(TAG, ">>> STT: %s", p);
    response(p);
    // TODO: Hiển thị LCD, gửi MQTT, xử lý lệnh...
    // lcd_print_line(0, "Nghe duoc:");
    // lcd_print_line(1, p);
}

// ================= SEND QUA WEBSOCKET =================
static void send_via_ws(int num_samples)
{
    // Kiểm tra kết nối
    EventBits_t bits = xEventGroupGetBits(ws_evt_grp);
    if (!(bits & WS_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WS chưa kết nối, bỏ qua");
        return;
    }

    // Xóa response bit cũ trước khi gửi
    xEventGroupClearBits(ws_evt_grp, WS_RESPONSE_BIT);

    // Gửi audio theo từng chunk binary
    uint8_t *ptr      = (uint8_t *)audio_buf;
    int      remaining = num_samples * sizeof(int16_t);
    int      chunk_no  = 0;

    while (remaining > 0) {
        int to_send = remaining > SEND_CHUNK ? SEND_CHUNK : remaining;

        int ret = esp_websocket_client_send_bin(
            ws_client, (char *)ptr, to_send, pdMS_TO_TICKS(5000)
        );

        if (ret < 0) {
            ESP_LOGE(TAG, "Gửi chunk %d thất bại", chunk_no);
            return;
        }

        ptr       += to_send;
        remaining -= to_send;
        chunk_no++;
    }

    ESP_LOGI(TAG, "Đã gửi %d chunks, gửi END...", chunk_no);

    // Gửi sentinel để server bắt đầu transcribe
    esp_websocket_client_send_text(
        ws_client, "END", 3, pdMS_TO_TICKS(1000)
    );

    // Chờ response tối đa 30s
    ESP_LOGI(TAG, "Chờ kết quả STT...");
    bits = xEventGroupWaitBits(
        ws_evt_grp, WS_RESPONSE_BIT,
        pdTRUE, pdTRUE,
        pdMS_TO_TICKS(30000)
    );

    if (bits & WS_RESPONSE_BIT) {
        handle_stt_result();
    } else {
        ESP_LOGW(TAG, "Timeout chờ STT response (30s)");
    }
}

// ================= MAIN TASK =================
void mic_task(void *pvParameters)
{
    // Cấp phát buffer từ PSRAM
    audio_buf = (int16_t *)heap_caps_malloc(
        MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM
    );
    if (!audio_buf) {
        ESP_LOGE(TAG, "PSRAM alloc thất bại!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "PSRAM buffer: %d KB",
             (int)(MAX_SAMPLES * sizeof(int16_t) / 1024));

    i2s_mic_init();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi I2S ổn định
    ws_init();  
    while (1) {
        int samples = record_with_vad();

        if (samples > SAMPLE_RATE / 2) {
            ESP_LOGI(TAG, "Gửi %d samples...", samples);
            send_via_ws(samples);
        } else {
            ESP_LOGW(TAG, "Clip quá ngắn (%.2fs), bỏ qua.",
                     (float)samples / SAMPLE_RATE);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}