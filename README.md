# Alzheimer Tremor Detection

An ESP32-based tremor detection system using the MPU6050 (GY-521) accelerometer/gyroscope. Designed to detect and log tremor events using acceleration magnitude and frequency analysis.

## Hardware

- ESP32
- GY-521 (MPU6050) — connected via I2C (SDA: GPIO 21, SCL: GPIO 22)
- Momentary push button — GPIO 23 (active LOW)
- LED — GPIO 2

## Sketches

### `tremor_with_button/`
Main detection sketch. Press the button to start/stop a logging session. During a session, the ESP32 samples the accelerometer at 100 Hz and logs tremor events to SPIFFS (on-device flash). An event is only recorded if it meets all three criteria:
- Duration ≥ 300 ms
- Average intensity ≥ 0.25 g
- Frequency ≥ 3 Hz (estimated from X-axis zero-crossings)

Each session is saved as `/session_NNN.txt` with CSV columns: `event_index, start_ms, end_ms, duration_ms, avg_intensity_g, freq_hz`.

### `test_mpu/`
Basic connectivity test. Verifies the MPU6050 `WHO_AM_I` register and streams raw accelerometer, gyroscope, and temperature readings to Serial at ~5 Hz. Use this first to confirm wiring is correct.

## Wiring

| MPU6050 | ESP32 |
|---------|-------|
| VCC     | 3.3V  |
| GND     | GND   |
| SDA     | GPIO 21 |
| SCL     | GPIO 22 |
| AD0     | GND (I2C addr 0x68) |
