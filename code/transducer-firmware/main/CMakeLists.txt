idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES json esp_http_server nvs_flash fatfs ulp arduino-esp32 HULP arduino-libraries)
