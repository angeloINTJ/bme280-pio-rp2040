/*
 * Final PIO I2C test - BMP280 via software GPIO I2C
 */
#include <Arduino.h>
#include "PIO_I2C.h"
#include "BMx280_PIO.h"

void setup() {
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200); delay(2000);
    Serial.println("=== BMx280 PIO Test ===\n");

    // Software I2C at 50kHz
    PIO_I2C i2c(2, 3, 50000);
    i2c.begin();

    // Scan
    i2c.scan();
    Serial.println();

    // Read chip ID
    uint8_t reg = 0xD0, val = 0;
    Serial.print("Chip ID: ");
    if (i2c.writeThenRead(0x76, &reg, 1, &val, 1)) {
        Serial.print("0x"); Serial.println(val, HEX);
    } else { Serial.println("FAIL"); }

    // Read calibration byte
    reg = 0x88;
    uint8_t cal[2];
    Serial.print("DIG_T1 (2 bytes): ");
    if (i2c.writeThenRead(0x76, &reg, 1, cal, 2)) {
        uint16_t T1 = cal[0] | (cal[1] << 8);
        Serial.print("0x"); Serial.print(cal[0],HEX);
        Serial.print(" 0x"); Serial.print(cal[1],HEX);
        Serial.print(" = "); Serial.println(T1);
    } else Serial.println("FAIL");

    // Now test full BMx280_PIO library
    Serial.println("\n--- BMx280_PIO Library ---");
    BMx280_PIO sensor(2, 3, 0x76, 50000);
    Serial.print("begin(): ");
    if (!sensor.begin()) {
        Serial.println("FAILED");
    } else {
        Serial.print("OK - ");
        Serial.println(sensor.isBME280() ? "BME280" : "BMP280");

        Serial.print("Reading sensor... ");
        sensor.setMode(BME280_MODE_FORCED);
        delay(50);
        float t = sensor.readTemperature();
        float p = sensor.readPressure();
        Serial.print("T="); Serial.print(t,2);
        Serial.print(" C  P="); Serial.print(p,2); Serial.println(" hPa");

        // Multiple readings
        Serial.println("\n5 readings:");
        for (int i = 0; i < 5; i++) {
            sensor.takeForcedMeasurement();
            sensor.readAll(&t, &p, nullptr);
            Serial.print(i); Serial.print(": T=");
            Serial.print(t,2); Serial.print(" P="); Serial.println(p,2);
            delay(200);
        }
    }

    i2c.end();
    Serial.println("\n=== DONE ===");
    digitalWrite(LED_BUILTIN, LOW);
}
void loop() { delay(1000); }
