/**
 * @example Basic Reading Example for BMx280PIO_RP2040
 * @brief
 * Reads temperature and pressure from a BMP280/BME280 sensor
 * using GPIO-based I2C on the RP2040. Outputs values to Serial.
 *
 * Wiring:
 *   Sensor VCC → 3.3V
 *   Sensor GND → GND
 *   Sensor SDA → GPIO2
 *   Sensor SCL → GPIO3
 *
 * Expected output (Serial Monitor, 115200 baud):
 *   BMP280 Test
 *   Temperature: 23.45 °C
 *   Pressure: 1013.25 hPa
 */

#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

// GPIO pins (any pair works)
#define SDA_PIN 2
#define SCL_PIN 3

BMx280PIO_RP2040 bme(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for USB Serial */ }

    Serial.println("BMx280 Test");
    Serial.println("===========");

    if (!bme.begin()) {
        Serial.println("ERROR: Sensor not found!");
        Serial.println("Check wiring:");
        Serial.println("  VCC -> 3.3V");
        Serial.println("  GND -> GND");
        Serial.println("  SDA -> GPIO2");
        Serial.println("  SCL -> GPIO3");
        while (1) { delay(1000); }
    }

    Serial.print("Sensor: ");
    Serial.println(bme.isBME280() ? "BME280" : "BMP280");
    Serial.print("Chip ID: 0x");
    Serial.println(bme.getChipID(), HEX);

    // Optional: configure for higher accuracy
    bme.setTemperatureOversampling(BME280_OS_2X);
    bme.setPressureOversampling(BME280_OS_4X);
    bme.setFilter(BME280_FILTER_4);
    bme.setMode(BME280_MODE_NORMAL);

    Serial.println("Sensor ready.\n");
}

void loop() {
    float temperature, pressure, humidity;
    bme.readAll(&temperature, &pressure, &humidity);

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("Pressure:    ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    if (bme.isBME280()) {
        Serial.print("Humidity:    ");
        Serial.print(humidity, 2);
        Serial.println(" %");
    }

    // Approximate altitude (standard atmosphere formula)
    float altitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f));
    Serial.print("Altitude:    ");
    Serial.print(altitude, 2);
    Serial.println(" m");

    Serial.println("------------------------");

    delay(2000);
}
