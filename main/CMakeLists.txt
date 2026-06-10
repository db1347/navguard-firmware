idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ekf spoofing nmea drivers ble_server ota)
