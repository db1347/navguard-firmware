# NavGuard IL firmware — dual-target build
#
# ESP32-S3 target (requires ESP-IDF v5.x environment):
#   idf.py set-target esp32s3 && idf.py build
#
# Desktop test target (laptop, no ESP32 needed):
#   mkdir build-desktop && cd build-desktop
#   cmake .. -DTARGET=desktop -DCMAKE_BUILD_TYPE=Debug
#   make -j4 && ctest --output-on-failure
cmake_minimum_required(VERSION 3.16)

# Allow either -DTARGET_PLATFORM=desktop or the dev-plan's -DTARGET=desktop.
if(DEFINED TARGET AND NOT DEFINED TARGET_PLATFORM)
    set(TARGET_PLATFORM "${TARGET}")
endif()

if(TARGET_PLATFORM STREQUAL "desktop")
    project(navguard_desktop C)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    add_compile_options(-Wall -Wextra -Werror)

    enable_testing()

    set(COMPONENTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/components)
    set(TEST_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
    add_compile_definitions(NAVGUARD_TEST_DATA_DIR="${TEST_DATA_DIR}")

    # ---- Pure-logic libraries (no hardware dependencies) ----
    add_library(nmea
        ${COMPONENTS_DIR}/nmea/nmea_parser.c
        ${COMPONENTS_DIR}/nmea/nmea_builder.c)
    target_include_directories(nmea PUBLIC ${COMPONENTS_DIR}/nmea)

    add_library(ekf
        ${COMPONENTS_DIR}/ekf/ekf.c
        ${COMPONENTS_DIR}/ekf/geo_math.c)
    target_include_directories(ekf PUBLIC ${COMPONENTS_DIR}/ekf)
    target_link_libraries(ekf PUBLIC nmea m)

    add_library(spoofing
        ${COMPONENTS_DIR}/spoofing/spoof_detect.c)
    target_include_directories(spoofing PUBLIC ${COMPONENTS_DIR}/spoofing)
    target_link_libraries(spoofing PUBLIC ekf nmea)

    # Mock driver backends + pure parsing helpers are desktop-testable
    add_library(drivers_mock
        ${COMPONENTS_DIR}/drivers/lc29h/lc29h_mock.c
        ${COMPONENTS_DIR}/drivers/bmi270/bmi270_mock.c
        ${COMPONENTS_DIR}/drivers/bmp390/bmp390_mock.c
        ${COMPONENTS_DIR}/drivers/sim7020e/sim7020e_mock.c
        ${COMPONENTS_DIR}/drivers/sim7020e/sim7020e_at_parser.c)
    target_include_directories(drivers_mock PUBLIC
        ${COMPONENTS_DIR}/drivers
        ${COMPONENTS_DIR}/drivers/lc29h
        ${COMPONENTS_DIR}/drivers/bmi270
        ${COMPONENTS_DIR}/drivers/bmp390
        ${COMPONENTS_DIR}/drivers/sim7020e)
    target_link_libraries(drivers_mock PUBLIC nmea ekf)

    # ---- Unit tests (CTest) ----
    function(navguard_add_test name source)
        add_executable(${name} ${source})
        target_link_libraries(${name} PRIVATE nmea ekf spoofing drivers_mock m)
        add_test(NAME ${name} COMMAND ${name})
    endfunction()

    navguard_add_test(test_nmea_parser  ${COMPONENTS_DIR}/nmea/test/test_nmea_parser.c)
    navguard_add_test(test_nmea_builder ${COMPONENTS_DIR}/nmea/test/test_nmea_builder.c)
    navguard_add_test(test_ekf          ${COMPONENTS_DIR}/ekf/test/test_ekf.c)
    navguard_add_test(test_spoof_detect ${COMPONENTS_DIR}/spoofing/test/test_spoof_detect.c)
    navguard_add_test(test_at_parser    ${COMPONENTS_DIR}/drivers/sim7020e/test/test_at_parser.c)

    # ---- Replay tool: NMEA file -> parser -> EKF -> spoof detect -> CSV ----
    add_executable(navguard_replay tools/replay_main.c)
    target_link_libraries(navguard_replay PRIVATE nmea ekf spoofing m)

else()
    # ESP-IDF build
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(navguard)
endif()
