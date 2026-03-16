idf_component_register(SRCS "cache_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        minimp3
                        config_parameter
                        esp_wifi
                        nvs_flash
                        esp_event)