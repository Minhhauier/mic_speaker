idf_component_register(SRCS "mic_stt_local.c"
                    INCLUDE_DIRS "include"
                    REQUIRES
                        esp_http_client
                        esp_driver_i2s)
                    