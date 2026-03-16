idf_component_register(
    SRCS "main_v2.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_http_client
        driver
        minimp3
        
        connect_wifi
        speaker
        cache_manager
        config_parameter
)