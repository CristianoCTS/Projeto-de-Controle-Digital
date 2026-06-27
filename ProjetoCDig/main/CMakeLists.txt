idf_component_register(
                    SRCS "main.c"
                    REQUIRES driver esp_adc esp_timer esp_driver_ledc
                    INCLUDE_DIRS ".")
