/**
 * @example Forced Mode Example for BMx280PIO_RP2040
 * @brief
 * Demonstrates Forced Mode for low-power operation.
 * The sensor takes a single measurement then returns to Sleep mode,
 * consuming minimal power between readings.
 *
 * Ideal for battery-powered weather stations and IoT sensors
 * that only need periodic readings.
 *
 * Wiring:
 *   Sensor VCC → 3.3V
 *   Sensor GND → GND
 *   Sensor SDA → GPIO2
 *   Sensor SCL → GPIO3
 */

#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

#define SDA_PIN 2
#define SCL_PIN 3
#define INTERVAL_MS 60000  // 1 minute between readings

BMx280PIO_RP2040 bme(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for USB Serial */ }

    Serial.println("BMx280 Forced Mode Example");
    Serial.println("==========================");

    if (!bme.begin()) {
        Serial.println("ERROR: Sensor not found!");
        while (1) delay(1000);
    }

    // Configure for low power: 1x oversampling, no filter
    bme.setTemperatureOversampling(BME280_OS_1X);
    bme.setPressureOversampling(BME280_OS_1X);
    bme.setFilter(BME280_FILTER_OFF);

    Serial.print("Sensor: ");
    Serial.println(bme.isBME280() ? "BME280" : "BMP280");
    Serial.print("Reading every ");
    Serial.print(INTERVAL_MS / 1000);
    Serial.println(" seconds...\n");
}

void loop() {
    Serial.print("Measuring... ");

    if (bme.takeForcedMeasurement()) {
        Serial.println("done!");

        float temperature = bme.readTemperature();
        float pressure    = bme.readPressure();

        Serial.print("Temperature: ");
        Serial.print(temperature, 2);
        Serial.println(" °C");

        Serial.print("Pressure:    ");
        Serial.print(pressure, 2);
        Serial.println(" hPa");

        if (bme.isBME280()) {
            float humidity = bme.readHumidity();
            Serial.print("Humidity:    ");
            Serial.print(humidity, 2);
            Serial.println(" %");

            // Dew point approximation (Magnus formula)
            float a = 17.27f, b = 237.7f;
            float gamma = (a * temperature) / (b + temperature)
                        + log(humidity / 100.0f);
            float dewPoint = (b * gamma) / (a - gamma);
            Serial.print("Dew Point:   ");
            Serial.print(dewPoint, 2);
            Serial.println(" °C");
        }

        // Approximate altitude
        float altitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f));
        Serial.print("Altitude:    ");
        Serial.print(altitude, 2);
        Serial.println(" m");

    } else {
        Serial.println("FAILED!");
    }

    Serial.print("Sleeping for ");
    Serial.print(INTERVAL_MS / 1000);
    Serial.println(" seconds...");
    Serial.println("------------------------");

    // Sensor automatically returns to sleep mode after forced measurement
    delay(INTERVAL_MS);
}
