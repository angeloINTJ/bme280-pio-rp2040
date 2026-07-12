/**
 * @example PIO+DMA Burst Read Example for BMx280PIO_RP2040
 * @brief
 * Demonstrates PIO+DMA burst reads for high-speed sensor readings.
 * The internal WirePIO handles PIO+DMA automatically — no manual
 * PIO setup needed. Uses Forced mode for low power.
 *
 * Wiring:
 *   Sensor VCC → 3.3V
 *   Sensor GND → GND
 *   Sensor SDA → GPIO2
 *   Sensor SCL → GPIO3
 */

#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

BMx280PIO_RP2040 bme(2, 3);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    delay(500);
    Serial.println("BMx280 PIO+DMA Burst Read");
    Serial.println("=========================");

    if (!bme.begin()) {
        Serial.println("ERROR: Sensor not found!");
        while (1) delay(1000);
    }
    Serial.print("Sensor: ");
    Serial.println(bme.isBME280() ? "BME280" : "BMP280");

    // begin() already initializes PIO+DMA via WirePIO internally.
    // No separate beginPIO() call needed.

    bme.setTemperatureOversampling(BME280_OS_1X);
    bme.setPressureOversampling(BME280_OS_1X);
    bme.setFilter(BME280_FILTER_OFF);

    Serial.println("PIO+DMA burst read active.\n");
}

void loop() {
    // Trigger measurement, then read all registers
    if (!bme.takeForcedMeasurement()) {
        Serial.println("Measurement failed!");
        delay(1000);
        return;
    }

    float t, p, h;
    bme.readAll(&t, &p, &h);  // Uses PIO+DMA burst via WirePIO

    Serial.print("T: "); Serial.print(t, 2); Serial.print(" C");
    Serial.print(" | P: "); Serial.print(p, 2); Serial.print(" hPa");
    if (bme.isBME280()) {
        Serial.print(" | H: "); Serial.print(h, 2); Serial.print(" %");
    }
    Serial.println();

    delay(2000);
}
