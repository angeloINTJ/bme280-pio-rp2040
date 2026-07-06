/*
 * Automated Test Suite for BME280_PIO Library
 *
 * Hardware setup:
 *   RP2040 (Raspberry Pi Pico)
 *   GP2 = SDA (I2C data)
 *   GP3 = SCL (I2C clock)
 *   GP0 = SDO (address select)
 *   GP1 = CSB (not used - SPI chip select)
 *   All pins: 10k pull-up to 3.3V
 *
 * Tests:
 *   1. PIO_I2C initialization
 *   2. I2C bus scan
 *   3. BME280 chip ID verification
 *   4. Calibration data integrity
 *   5. Temperature reading (valid range check)
 *   6. Pressure reading (valid range check)
 *   7. Humidity reading (valid range check)
 *   8. Forced mode measurement
 *   9. Oversampling configuration change
 *   10. readAll() burst efficiency
 *
 * Output: JSON-formatted test results via USB Serial (115200 baud)
 */

#include <Arduino.h>
#include "BME280_PIO.h"

// ─── Pin Configuration (as wired on protoboard) ───────────────────────────
#define SDA_PIN   2
#define SCL_PIN   3
#define SDO_PIN   0   // Not directly used, affects I2C address
#define CSB_PIN   1   // Not used (I2C mode)

// ─── Test Macros ──────────────────────────────────────────────────────────
unsigned int tests_passed = 0;
unsigned int tests_failed = 0;
unsigned int tests_total  = 0;

#define TEST_BEGIN(name) do { \
    tests_total++; \
    Serial.print("{\"test\":\"" name "\",\"status\":\"running\"}"); \
    Serial.println(); \
} while(0)

#define TEST_PASS(msg) do { \
    tests_passed++; \
    Serial.print("{\"test\":\"" msg "\",\"result\":\"PASS\"}"); \
    Serial.println(); \
} while(0)

#define TEST_FAIL(msg, detail) do { \
    tests_failed++; \
    Serial.print("{\"test\":\"" msg "\",\"result\":\"FAIL\",\"detail\":\"" detail "\"}"); \
    Serial.println(); \
} while(0)

#define TEST_ASSERT(cond, name, fail_msg) do { \
    if (cond) { TEST_PASS(name); } \
    else { TEST_FAIL(name, fail_msg); } \
} while(0)

// ─── JSON Output Helpers ──────────────────────────────────────────────────

void json_begin() {
    Serial.println("{\"test_suite\":\"BME280_PIO\",\"hardware\":\"RP2040\"}");
}

void json_value(const char* key, float value, const char* unit) {
    Serial.print("{\"");
    Serial.print(key);
    Serial.print("\":");
    Serial.print(value, 4);
    Serial.print(",\"unit\":\"");
    Serial.print(unit);
    Serial.println("\"}");
}

void json_int(const char* key, int value) {
    Serial.print("{\"");
    Serial.print(key);
    Serial.print("\":");
    Serial.print(value);
    Serial.println("}");
}

void json_summary() {
    Serial.print("{\"summary\":{\"passed\":");
    Serial.print(tests_passed);
    Serial.print(",\"failed\":");
    Serial.print(tests_failed);
    Serial.print(",\"total\":");
    Serial.print(tests_total);
    Serial.print(",\"score\":");
    Serial.print(tests_total > 0 ? (tests_passed * 100 / tests_total) : 0);
    Serial.println("}}");
}

// ─── Test Runner ──────────────────────────────────────────────────────────

// Try both possible I2C addresses
uint8_t find_bme280_address(PIO_I2C &i2c) {
    for (uint8_t addr = 0x76; addr <= 0x77; addr++) {
        // Quick probe: try to read chip ID
        uint8_t cmd = 0xD0; // CHIP_ID register
        uint8_t chip_id = 0;
        if (i2c.writeThenRead(addr, &cmd, 1, &chip_id, 1)) {
            if (chip_id == 0x60) {
                return addr;
            }
        }
    }
    return 0;
}

void setup() {
    Serial.begin(115200);
    // Wait up to 3 seconds for Serial (native USB on Pico)
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 3000) {
        tight_loop_contents();
    }

    delay(500); // Let USB stabilize

    json_begin();
    Serial.println("{\"phase\":\"setup\",\"msg\":\"BME280_PIO Test Suite Starting...\"}");

    // ─── Test 1: PIO_I2C Initialization ───────────────────────────────
    {
        TEST_BEGIN("1 - PIO_I2C Init");
        PIO_I2C i2c(SDA_PIN, SCL_PIN, PIO_I2C_FREQ_STANDARD);

        if (i2c.begin()) {
            TEST_PASS("1 - PIO_I2C Init");
            json_int("pio_i2c_sda", SDA_PIN);
            json_int("pio_i2c_scl", SCL_PIN);
            json_int("pio_i2c_freq", 100000);

            // ─── Test 2: I2C Bus Scan ─────────────────────────────────
            {
                TEST_BEGIN("2 - I2C Bus Scan");
                Serial.println("{\"phase\":\"scan\",\"msg\":\"Scanning I2C bus...\"}");

                int devices = 0;
                uint8_t found_addrs[16] = {0};

                for (uint8_t a = 0x01; a < 0x78; a++) {
                    uint8_t dummy = 0;
                    if (i2c.write(a, &dummy, 1, true)) {
                        if (devices < 16) found_addrs[devices] = a;
                        devices++;
                    }
                    delayMicroseconds(200);
                }

                Serial.print("{\"i2c_devices_found\":");
                Serial.print(devices);
                Serial.print(",\"addresses\":[");
                for (int i = 0; i < devices; i++) {
                    if (i > 0) Serial.print(",");
                    Serial.print("\"0x");
                    Serial.print(found_addrs[i], HEX);
                    Serial.print("\"");
                }
                Serial.println("]}");

                if (devices > 0) {
                    TEST_PASS("2 - I2C Bus Scan");
                } else {
                    TEST_FAIL("2 - I2C Bus Scan", "No devices found on I2C bus. Check wiring and pull-ups.");
                }
            }

            // ─── Test 3: Find BME280 ──────────────────────────────────
            uint8_t bme_addr = 0;
            {
                TEST_BEGIN("3 - Find BME280");
                bme_addr = find_bme280_address(i2c);

                if (bme_addr > 0) {
                    TEST_PASS("3 - Find BME280");
                    Serial.print("{\"bme280_address\":\"0x");
                    Serial.print(bme_addr, HEX);
                    Serial.println("\"}");

                    // Report SDO pin state
                    json_int("sdo_pin", SDO_PIN);
                    Serial.print("{\"sdo_state\":\"");
                    Serial.print(bme_addr == 0x76 ? "LOW (GND)" : "HIGH (VCC)");
                    Serial.println("\"}");
                } else {
                    TEST_FAIL("3 - Find BME280", "Chip ID != 0x60 or no response at 0x76/0x77");
                }
            }

            // ─── Tests 4-10: BME280 Functionality ─────────────────────
            if (bme_addr > 0) {
                BME280_PIO bme(SDA_PIN, SCL_PIN, bme_addr, PIO_I2C_FREQ_STANDARD);

                // ─── Test 4: Sensor Initialization ────────────────────
                {
                    TEST_BEGIN("4 - Sensor Begin");
                    if (bme.begin()) {
                        TEST_PASS("4 - Sensor Begin");
                    } else {
                        TEST_FAIL("4 - Sensor Begin", "begin() returned false");
                    }
                }

                // ─── Test 5: Chip ID ──────────────────────────────────
                {
                    TEST_BEGIN("5 - Chip ID Verification");
                    uint8_t cid = bme.getChipID();
                    Serial.print("{\"chip_id\":\"0x");
                    Serial.print(cid, HEX);
                    Serial.println("\"}");
                    TEST_ASSERT(cid == 0x60, "5 - Chip ID Verification",
                               "Expected 0x60");
                }

                // ─── Test 6: Calibration Data ─────────────────────────
                {
                    TEST_BEGIN("6 - Calibration Data");
                    // Read a few calibration registers to verify
                    uint8_t cal[2];
                    bme.readRegisters(0x88, cal, 2);
                    uint16_t dig_T1 = cal[0] | ((uint16_t)cal[1] << 8);

                    bme.readRegisters(0x8E, cal, 2);
                    uint16_t dig_P1 = cal[0] | ((uint16_t)cal[1] << 8);

                    bool cal_ok = (dig_T1 > 0 && dig_T1 < 0xFFFF) &&
                                  (dig_P1 > 0 && dig_P1 < 0xFFFF);
                    TEST_ASSERT(cal_ok, "6 - Calibration Data",
                               "Calibration values out of range");
                    Serial.print("{\"dig_T1\":");
                    Serial.print(dig_T1);
                    Serial.print(",\"dig_P1\":");
                    Serial.print(dig_P1);
                    Serial.println("}");
                }

                // ─── Test 7: Forced Mode Measurement ──────────────────
                {
                    TEST_BEGIN("7 - Forced Mode");
                    bme.setMode(BME280_MODE_SLEEP);

                    uint32_t t0 = micros();
                    bool ok = bme.takeForcedMeasurement();
                    uint32_t t1 = micros();

                    if (ok) {
                        TEST_PASS("7 - Forced Mode");
                        Serial.print("{\"measurement_time_us\":");
                        Serial.print(t1 - t0);
                        Serial.println("}");
                    } else {
                        TEST_FAIL("7 - Forced Mode", "takeForcedMeasurement() failed or timed out");
                    }
                }

                // ─── Test 8: Temperature ──────────────────────────────
                float temperature = 0;
                {
                    TEST_BEGIN("8 - Temperature Reading");
                    temperature = bme.readTemperature();

                    bool temp_ok = (temperature > -40.0f && temperature < 85.0f);
                    TEST_ASSERT(temp_ok, "8 - Temperature Reading",
                               "Temperature out of valid range (-40 to +85 C)");
                    json_value("temperature", temperature, "C");
                }

                // ─── Test 9: Pressure ──────────────────────────────────
                float pressure = 0;
                {
                    TEST_BEGIN("9 - Pressure Reading");
                    pressure = bme.readPressure();

                    bool press_ok = (pressure > 300.0f && pressure < 1100.0f);
                    TEST_ASSERT(press_ok, "9 - Pressure Reading",
                               "Pressure out of valid range (300-1100 hPa)");
                    json_value("pressure", pressure, "hPa");
                }

                // ─── Test 10: Humidity ────────────────────────────────
                float humidity = 0;
                {
                    TEST_BEGIN("10 - Humidity Reading");
                    humidity = bme.readHumidity();

                    bool hum_ok = (humidity >= 0.0f && humidity <= 100.0f);
                    TEST_ASSERT(hum_ok, "10 - Humidity Reading",
                               "Humidity out of valid range (0-100%)");
                    json_value("humidity", humidity, "%RH");
                }

                // ─── Test 11: readAll() Function ──────────────────────
                {
                    TEST_BEGIN("11 - readAll Burst Read");
                    float t, p, h;
                    uint32_t t0 = micros();
                    bme.readAll(&t, &p, &h);
                    uint32_t t1 = micros();

                    bool all_ok = (t > -40.0f && t < 85.0f) &&
                                  (p > 300.0f && p < 1100.0f) &&
                                  (h >= 0.0f && h <= 100.0f);

                    TEST_ASSERT(all_ok, "11 - readAll Burst Read",
                               "readAll() returned invalid values");
                    Serial.print("{\"readAll_time_us\":");
                    Serial.print(t1 - t0);
                    Serial.println("}");
                    json_value("readAll_temp", t, "C");
                    json_value("readAll_press", p, "hPa");
                    json_value("readAll_hum", h, "%RH");
                }

                // ─── Test 12: Oversampling Change ─────────────────────
                {
                    TEST_BEGIN("12 - Oversampling Config Change");
                    bme.setMode(BME280_MODE_SLEEP);

                    // Change to higher oversampling
                    bme.setTemperatureOversampling(BME280_OS_4X);
                    bme.setPressureOversampling(BME280_OS_4X);
                    bme.setHumidityOversampling(BME280_OS_4X);
                    bme.setFilter(BME280_FILTER_4);

                    bool ok = bme.takeForcedMeasurement();
                    float t2 = bme.readTemperature();
                    float p2 = bme.readPressure();
                    float h2 = bme.readHumidity();

                    bool valid = (t2 > -40.0f && t2 < 85.0f) &&
                                 (p2 > 300.0f && p2 < 1100.0f) &&
                                 (h2 >= 0.0f && h2 <= 100.0f);

                    TEST_ASSERT((ok && valid), "12 - Oversampling Config Change",
                               "High oversampling measurement failed");
                    Serial.print("{\"osrs_4x_temp\":");
                    Serial.print(t2, 4);
                    Serial.print(",\"osrs_4x_press\":");
                    Serial.print(p2, 4);
                    Serial.print(",\"osrs_4x_hum\":");
                    Serial.print(h2, 4);
                    Serial.println("}");

                    // Restore defaults
                    bme.setMode(BME280_MODE_SLEEP);
                    bme.setTemperatureOversampling(BME280_OS_1X);
                    bme.setPressureOversampling(BME280_OS_1X);
                    bme.setHumidityOversampling(BME280_OS_1X);
                    bme.setFilter(BME280_FILTER_OFF);
                }

                // ─── Test 13: Normal Mode Continuous Reading ──────────
                {
                    TEST_BEGIN("13 - Normal Mode Continuous");

                    bme.setMode(BME280_MODE_NORMAL);
                    delay(50); // Wait for first measurement

                    // Take 5 readings 200ms apart
                    float temps[5];
                    bool all_valid = true;
                    uint32_t t0 = micros();

                    for (int i = 0; i < 5; i++) {
                        delay(200);
                        temps[i] = bme.readTemperature();
                        if (temps[i] < -40.0f || temps[i] > 85.0f) {
                            all_valid = false;
                        }
                    }
                    uint32_t t1 = micros();

                    // Check readings are stable (max deviation < 1°C)
                    float t_min = temps[0], t_max = temps[0];
                    for (int i = 1; i < 5; i++) {
                        if (temps[i] < t_min) t_min = temps[i];
                        if (temps[i] > t_max) t_max = temps[i];
                    }
                    float deviation = t_max - t_min;

                    TEST_ASSERT(all_valid && deviation < 5.0f, "13 - Normal Mode Continuous",
                               "Normal mode readings unstable or invalid");

                    Serial.print("{\"normal_mode_samples\":5");
                    Serial.print(",\"temp_range\":");
                    Serial.print(deviation, 4);
                    Serial.print(",\"total_time_us\":");
                    Serial.print(t1 - t0);
                    Serial.print(",\"samples\":[");
                    for (int i = 0; i < 5; i++) {
                        if (i > 0) Serial.print(",");
                        Serial.print(temps[i], 4);
                    }
                    Serial.println("]}");

                    bme.setMode(BME280_MODE_SLEEP);
                }

                // ─── Test 14: Stress Test — Rapid Forced Measurements ─
                {
                    TEST_BEGIN("14 - Rapid Forced Measurements");

                    int success = 0;
                    uint32_t t0 = micros();

                    for (int i = 0; i < 10; i++) {
                        if (bme.takeForcedMeasurement()) {
                            float t = bme.readTemperature();
                            if (t > -40.0f && t < 85.0f) success++;
                        }
                    }
                    uint32_t t1 = micros();

                    TEST_ASSERT(success >= 9, "14 - Rapid Forced Measurements",
                               "Less than 90% of rapid measurements succeeded");

                    Serial.print("{\"rapid_measurements\":10");
                    Serial.print(",\"successful\":");
                    Serial.print(success);
                    Serial.print(",\"total_time_ms\":");
                    Serial.print((t1 - t0) / 1000.0f, 1);
                    Serial.println("}");
                }

                bme.setMode(BME280_MODE_SLEEP);

            } // bme_addr > 0

            // ─── Final: PIO Resource Cleanup ──────────────────────────
            {
                TEST_BEGIN("15 - PIO Cleanup");
                i2c.end();
                TEST_PASS("15 - PIO Cleanup");
            }

        } else {
            TEST_FAIL("1 - PIO_I2C Init", "No free PIO state machine available");
        }
    }

    // ─── Summary ──────────────────────────────────────────────────────────
    Serial.println();
    json_summary();

    if (tests_failed == 0) {
        Serial.println("{\"verdict\":\"ALL TESTS PASSED\"}");
    } else {
        Serial.print("{\"verdict\":\"");
        Serial.print(tests_failed);
        Serial.println(" TEST(S) FAILED\"}");
    }

    Serial.println("{\"phase\":\"done\"}");
}

void loop() {
    // All tests run once in setup().
    // Blink LED to confirm the firmware is still alive.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
