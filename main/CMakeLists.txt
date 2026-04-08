idf_component_register(SRCS "dvp_spi_lcd_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_lcd esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_spi esp_timer)
