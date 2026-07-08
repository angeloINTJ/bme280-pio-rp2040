/**
 * @example PIO+DMA Auto-Scan Example for BMx280PIO_RP2040
 * @brief
 * Demonstrates PIO+DMA burst reads for high-speed sensor readings.
 * Uses the RP2040 PIO state machine to handle I2C transactions
 * with DMA for zero-CPU data transfer.
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

    // Load PIO program — enables PIO+DMA transport via burstRead()
    if (!bme.beginPIO(pio0)) {
        Serial.println("ERROR: PIO load failed!");
        while (1) delay(1000);
    }

    bme.setTemperatureOversampling(BME280_OS_1X);
    bme.setPressureOversampling(BME280_OS_1X);
    bme.setFilter(BME280_FILTER_OFF);

    Serial.println("PIO+DMA burst read active.\n");
}

void loop() {
    // Trigger measurement via GPIO, then read all registers
    // via PIO+DMA burst (burstRead handles the I2C transaction)
    if (!bme.takeForcedMeasurement()) {
        Serial.println("Measurement failed!");
        delay(1000);
        return;
    }

    float t, p, h;
    bme.readAll(&t, &p, &h);  // Uses burstRead() when PIO is active

    Serial.print("T: "); Serial.print(t, 2); Serial.print(" °C");
    Serial.print(" | P: "); Serial.print(p, 2); Serial.print(" hPa");
    if (bme.isBME280()) {
        Serial.print(" | H: "); Serial.print(h, 2); Serial.print(" %");
    }
    Serial.println();

    delay(2000);
}
