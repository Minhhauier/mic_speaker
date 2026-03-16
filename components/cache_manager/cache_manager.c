#include <string.h>
#include <stdio.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_log.h>

#include "cache_manager.h"
#include "config_parameter.h"

static const char *TAG = "cache_manager";

static uint8_t *mp3_data = NULL;
static size_t mp3_len = 0;
static size_t mp3_cap = 0;
static bool     tts_cache_enabled = true;

uint32_t fnv1a_32(const char *s)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; s[i] != '\0'; i++) {
        h ^= (uint8_t)s[i];
        h *= 16777619u;
    }
    return h;
}

bool tts_cache_load(const char *text)
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

bool ensure_mp3_capacity(size_t needed)
{
    if (needed == 0) return false;
    if (mp3_data != NULL && mp3_cap >= needed) return true;

    uint8_t *tmp = realloc(mp3_data, needed);
    if (!tmp) return false;

    mp3_data = tmp;
    mp3_cap = needed;
    return true;
}

void tts_cache_store(const char *text, const uint8_t *data, size_t len)
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