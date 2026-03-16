#ifndef CONFIG_PARAMETER_H
#define CONFIG_PARAMETER_H

#define WIFI_SSID "ABC"
#define WIFI_PASS "coinhe2018"
#define WIFI_MAX_RETRY 5
// cấu hình chân I2S
#define I2S_BCLK_PIN GPIO_NUM_27
#define I2S_LRC_PIN GPIO_NUM_14
#define I2S_DOUT_PIN GPIO_NUM_13
// cấu hình audio
#define SAMPLE_RATE 22050
#define I2S_PORT I2S_NUM_0
#define HTTP_BUF_SIZE (4 * 1024)
#define MP3_BUF_SIZE (64 * 1024)
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

#endif