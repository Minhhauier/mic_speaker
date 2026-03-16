#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <minimp3.h>

#include "connect_wifi.h"
#include "speaker.h"
// #include "cache_manager.h"
#include "config_parameter.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Khởi động ứng dụng...");

    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGW(TAG, "Reset reason: %d", (int)reason); // check reset reason at startup

    ESP_LOGI(TAG, "=== ESP32 Tiếng Việt TTS (ESP-IDF v5.x) ===");
    ESP_LOGI(TAG, "Heap khả dụng: %d bytes", (int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Volume: %d%%", AUDIO_VOLUME_PERCENT);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    printf("wifi đã được khởi tạo.\n");
    i2s_init();
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    xTaskCreate(speaker_task, "test_audio_task", 1024*24, NULL, 5, NULL);
    
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}