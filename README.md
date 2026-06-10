# navguard-firmware

GPS anti-spoofing firmware for the NavGuard IL device (ESP32-S3, ESP-IDF 5.x).
Reads GPS/IMU/cell/barometer, fuses them in an Extended Kalman Filter, detects
spoofing, and outputs clean NMEA over BLE to the companion Android app.

The pure-logic core (NMEA, EKF, spoofing detector, AT parser) is **fully
testable on a laptop with CMake — no ESP32 required.** Hardware drivers have a
mock backend (desktop) and a real backend (`#ifdef ESP_PLATFORM`) behind one
interface, so logic is validated before hardware arrives.

## Layout

```
components/
  nmea/        NMEA 0183 parser + builder            (desktop-tested)
  ekf/         Extended Kalman Filter + geo math      (desktop-tested)
  spoofing/    4-state spoofing detector              (desktop-tested)
  drivers/     lc29h (GPS/UART), bmi270 (IMU/SPI),
               bmp390 (baro/I2C), sim7020e (cell/UART) — mock + real backends
  ble_server/  Nordic UART Service GATT server (NimBLE)
  ota/         WiFi HTTPS OTA update
main/main.c    FreeRTOS task wiring (imu/gps/cell/baro/output)
tools/         replay harness + NMEA generator + EKF plotter
test_data/     synthetic Tel Aviv tracks (normal + spoofing)
```

## Desktop tests (no hardware)

```bash
mkdir build-desktop && cd build-desktop
cmake .. -DTARGET=desktop -DCMAKE_BUILD_TYPE=Debug   # or -G Ninja
cmake --build .
ctest --output-on-failure
```

Five suites run: `test_nmea_parser`, `test_nmea_builder`, `test_ekf`,
`test_spoof_detect`, `test_at_parser`.

> macOS without a system CMake: the Android SDK ships one. Add it to PATH:
> `export PATH="$HOME/Library/Android/sdk/cmake/3.22.1/bin:$PATH"`

### Replay a track through the full pipeline

```bash
./build-desktop/navguard_replay test_data/tel_aviv_spoof.nmea > out.csv
python3 tools/plot_ekf.py out.csv     # matplotlib track plot, or text summary
```

On the spoofing track the detector transitions `LOCK → SUSPECT → SPOOFED`
within 200 ms of the Beirut jump, and the EKF holds position in Israel.

### Generate test data

```bash
python3 tools/gen_test_nmea.py            > test_data/tel_aviv_drive.nmea
python3 tools/gen_test_nmea.py --spoof    > test_data/tel_aviv_spoof.nmea
```

Replace these with real recordings from the **GPS Logger** app (NMEA export)
when available — real multipath/HDOP/timing exercises more edge cases.

## ESP32-S3 build (when hardware is present)

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py build flash monitor
```

Driver selection lives in `main/main.c`. Bring-up swaps each mock for its real
backend one at a time:

```c
s_gps  = lc29h_get_uart_driver();      // was lc29h_get_mock_driver(...)
s_imu  = bmi270_get_spi_driver();
s_baro = bmp390_get_i2c_driver();
s_cell = sim7020e_get_uart_driver();
```

Pin assignments and bring-up order are in the dev plan (Section 8).

## Spoofing detector

| State | GPS weight | Meaning |
|-------|-----------|---------|
| `GPS_LOCK` | 0.85 | all checks pass |
| `GPS_SUSPECT` | 0.20 | one check failed, evaluating |
| `GPS_SPOOFED` | 0.00 | cell anchor / teleport contradicts GPS |
| `DR_ONLY` | 0.00 | no GPS (tunnel / jam) — dead reckoning |

Checks: an **innovation velocity gate** (GPS jump may exceed EKF-expected travel
by ≤50 km/h, catching teleports while tolerating highway speed), a **heading
consistency** check (GPS turn vs gyro turn), and the definitive **cell anchor**
(serving tower MCC ≠ 425 while GPS claims Israel ⇒ spoofed).
