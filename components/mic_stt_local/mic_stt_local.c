// #include <stdio.h>
// #include <string.h>
// #include "esp_log.h"
// #include "esp_http_client.h"
// #include "driver/i2s_std.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2s_std.h"
// #include "esp_log.h"

// #define SERVER_URL       "http://192.168.1.100:8080/transcribe"
// #define BOUNDARY         "----ESP32Boundary"

// #define RECORD_SECONDS   3
// #define SAMPLE_RATE      16000
// #define TOTAL_SAMPLES    (SAMPLE_RATE * RECORD_SECONDS)

// #define RESP_BUF_SIZE    2048
// #define SEND_CHUNK       4096

// static const char *TAG = "mic_stt";
// i2s_chan_handle_t rx_handle = NULL;
// // ⚠️ nếu thiếu RAM → giảm RECORD_SECONDS hoặc dùng PSRAM
// static int16_t audio_buf[TOTAL_SAMPLES];
// static char response_buf[RESP_BUF_SIZE];
// static int response_len = 0;

// void i2s_init(void)
// {
//     // ================= CHANNEL CONFIG =================
//     i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
//         I2S_NUM_0,
//         I2S_ROLE_MASTER
//     );

//     ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

//     // ================= STD CONFIG =================
//     i2s_std_config_t std_cfg = {
//         .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000), // 16kHz

//         .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
//             I2S_DATA_BIT_WIDTH_32BIT,   // INMP441 xuất 24bit trong 32bit
//             I2S_SLOT_MODE_MONO
//         ),

//         .gpio_cfg = {
//             .mclk = I2S_GPIO_UNUSED,
//             .bclk = GPIO_NUM_33,  // SCK
//             .ws   = GPIO_NUM_25,  // WS
//             .dout = I2S_GPIO_UNUSED,
//             .din  = GPIO_NUM_32,  // SD
//             .invert_flags = {
//                 .mclk_inv = false,
//                 .bclk_inv = false,
//                 .ws_inv   = false,
//             },
//         },
//     };

//     ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));

//     // ================= ENABLE =================
//     ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

//     ESP_LOGI(TAG, "I2S init done");
// }

// // ================= WAV HEADER =================
// static void build_wav_header(uint8_t *header, int samples, int sample_rate)
// {
//     int byte_rate = sample_rate * 2; // 16-bit mono
//     int data_size = samples * 2;

//     memcpy(header, "RIFF", 4);
//     *(uint32_t *)(header + 4) = 36 + data_size;
//     memcpy(header + 8, "WAVE", 4);

//     memcpy(header + 12, "fmt ", 4);
//     *(uint32_t *)(header + 16) = 16;
//     *(uint16_t *)(header + 20) = 1;  // PCM
//     *(uint16_t *)(header + 22) = 1;  // mono
//     *(uint32_t *)(header + 24) = sample_rate;
//     *(uint32_t *)(header + 28) = byte_rate;
//     *(uint16_t *)(header + 32) = 2;
//     *(uint16_t *)(header + 34) = 16;

//     memcpy(header + 36, "data", 4);
//     *(uint32_t *)(header + 40) = data_size;
// }

// // ================= RECORD AUDIO =================
// extern i2s_chan_handle_t rx_handle;

// static void record_audio(void)
// {
//     int32_t raw;
//     size_t bytes_read;

//     for (int i = 0; i < TOTAL_SAMPLES; i++) {

//         esp_err_t ret = i2s_channel_read(
//             rx_handle,
//             &raw,
//             sizeof(raw),
//             &bytes_read,
//             portMAX_DELAY
//         );

//         if (ret != ESP_OK || bytes_read != sizeof(raw)) {
//             i--; // đọc lại sample này
//             continue;
//         }

//         // Convert 32-bit → 16-bit
//         audio_buf[i] = (int16_t)(raw >> 14); // gain nhẹ
//     }
// }

// // ================= HTTP EVENT =================
// static esp_err_t http_event_handler(esp_http_client_event_t *evt)
// {
//     if (evt->event_id == HTTP_EVENT_ON_DATA) {
//         int copy_len = evt->data_len;

//         if (response_len + copy_len >= RESP_BUF_SIZE - 1) {
//             copy_len = RESP_BUF_SIZE - response_len - 1;
//         }

//         if (copy_len > 0) {
//             memcpy(response_buf + response_len, evt->data, copy_len);
//             response_len += copy_len;
//             response_buf[response_len] = '\0';
//         }
//     }
//     return ESP_OK;
// }

// // ================= SEND TO SERVER =================
// static void send_to_server(void)
// {
//     uint8_t wav_hdr[44];
//     build_wav_header(wav_hdr, TOTAL_SAMPLES, SAMPLE_RATE);

//     uint32_t audio_bytes = TOTAL_SAMPLES * sizeof(int16_t);

//     const char *part_hdr =
//         "--" BOUNDARY "\r\n"
//         "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n"
//         "Content-Type: audio/wav\r\n\r\n";

//     const char *part_end =
//         "\r\n--" BOUNDARY "--\r\n";

//     int content_length =
//         strlen(part_hdr) + 44 + audio_bytes + strlen(part_end);

//     response_len = 0;
//     memset(response_buf, 0, sizeof(response_buf));

//     esp_http_client_config_t config = {
//         .url = SERVER_URL,
//         .method = HTTP_METHOD_POST,
//         .event_handler = http_event_handler,
//         .timeout_ms = 120000,
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);

//     esp_http_client_set_header(
//         client,
//         "Content-Type",
//         "multipart/form-data; boundary=" BOUNDARY
//     );

//     ESP_ERROR_CHECK(esp_http_client_open(client, content_length));

//     // gửi header multipart
//     esp_http_client_write(client, part_hdr, strlen(part_hdr));

//     // gửi WAV header
//     esp_http_client_write(client, (char *)wav_hdr, 44);

//     // gửi audio theo chunk
//     uint8_t *ptr = (uint8_t *)audio_buf;
//     int remaining = audio_bytes;

//     while (remaining > 0) {
//         int to_send = (remaining > SEND_CHUNK) ? SEND_CHUNK : remaining;

//         esp_http_client_write(client, (char *)ptr, to_send);

//         ptr += to_send;
//         remaining -= to_send;
//     }

//     // kết thúc multipart
//     esp_http_client_write(client, part_end, strlen(part_end));

//     esp_http_client_fetch_headers(client);

//     int status = esp_http_client_get_status_code(client);

//     ESP_LOGI(TAG, "HTTP %d", status);
//     ESP_LOGI(TAG, "Response: %s", response_buf);

//     // parse đơn giản
//     char *p = strstr(response_buf, "\"text\":\"");
//     if (p) {
//         p += 8;
//         char *end = strchr(p, '"');
//         if (end) *end = '\0';

//         ESP_LOGI(TAG, ">>> STT: %s", p);
//     }

//     esp_http_client_cleanup(client);
// }

// // ================= MAIN =================
// // void app_main(void)
// // {
// //     // i2s_init();
// //     // wifi_init();

// //     while (1) {

// //         ESP_LOGI(TAG, "Recording...");
// //         record_audio();

// //         ESP_LOGI(TAG, "Sending...");
// //         send_to_server();

// //         vTaskDelay(pdMS_TO_TICKS(2000));
// //     }
// // }