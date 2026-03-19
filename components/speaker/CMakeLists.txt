idf_component_register(SRCS "speaker.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                        esp_http_client
                        esp_driver_i2s

                        minimp3
                        config_parameter
                        esp_wifi
                        cache_manager
                        mpu6050)