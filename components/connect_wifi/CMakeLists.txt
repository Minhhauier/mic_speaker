idf_component_register(SRCS "connect_wifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES
                        config_parameter
                        esp_wifi)