/**
 * @file standalone_test.ino
 * @brief BMx280PIO_RP2040 hardware test example.
 *
 * Tests BMP280/BME280 sensor reading using the WirePIO transport.
 * Reads temperature, pressure, and humidity (if BME280) every 500ms.
 *
 * Wiring:
 *   VCC → 3.3V, GND → GND, SDA → GPIO4, SCL → GPIO5
 *
 * Output: Serial Monitor at 115200 baud.
 */

#include "BMx280PIO_RP2040.h"

BMx280PIO_RP2040 bme(4, 5);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    delay(500);

    if (!bme.begin()) {
        Serial.println("FAIL: Sensor not found!");
        while (1);
    }

    Serial.print("Sensor: ");
    Serial.println(bme.isBME280() ? "BME280" : "BMP280");

    bme.setMode(BME280_MODE_NORMAL);
    delay(100);

    float t, p, h;
    for (int i = 0; i < 5; i++) {
        bme.readAll(&t, &p, &h);
        Serial.print("T=");
        Serial.print(t, 2);
        Serial.print(" C  P=");
        Serial.print(p, 2);
        Serial.println(" hPa");
        delay(500);
    }
}

void loop() {
    float t, p, h;
    bme.readAll(&t, &p, &h);
    Serial.print("T=");
    Serial.print(t, 2);
    Serial.print(" C  P=");
    Serial.print(p, 2);
    Serial.println(" hPa");
    delay(500);
}
