# All driver backends. The *_mock.c files build everywhere; the hardware
# backends (*_uart.c, *_spi.c, *_i2c.c) are guarded by #ifdef ESP_PLATFORM so
# they only produce code on the ESP32 build.
idf_component_register(
    SRCS
        "lc29h/lc29h_mock.c"   "lc29h/lc29h_uart.c"
        "bmi270/bmi270_mock.c" "bmi270/bmi270_spi.c"
        "bmp390/bmp390_mock.c" "bmp390/bmp390_i2c.c"
        "sim7020e/sim7020e_mock.c" "sim7020e/sim7020e_uart.c"
        "sim7020e/sim7020e_at_parser.c"
    INCLUDE_DIRS
        "." "lc29h" "bmi270" "bmp390" "sim7020e"
    REQUIRES ekf nmea driver esp_driver_uart esp_driver_spi esp_driver_i2c)
