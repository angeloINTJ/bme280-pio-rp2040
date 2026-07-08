/*
 * PIO+DMA Auto-Scan Example for BMx280PIO_RP2040
 *
 * ⚠️ EXPERIMENTAL — The DMA ring buffer extraction is not yet fully
 * validated on hardware. Use basic_reading or forced_mode for
 * production. This example demonstrates the architecture and
 * compiles correctly, but readAllAsync() returns incorrect data.
 *
 * Demonstrates zero-CPU-overhead continuous sampling using the
 * 3-channel DMA engine (TX + RX + CTRL pacer).
 *
 * The PIO state machine executes I2C bursts autonomously.
 * DMA CH1 feeds commands, CH2 collects data to a ring buffer,
 * and CH3 (triggered by PWM) restarts CH1 each period.
 * The CPU only calls readAllAsync() when it needs fresh data —
 * all I2C bit-banging runs in background with zero CPU involvement.
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
#define SCAN_PERIOD_MS 1000  // Read sensor every 1 second

BMx280PIO_RP2040 bme(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for USB Serial */ }

    Serial.println("BMx280 PIO+DMA Auto-Scan");
    Serial.println("=========================");

    // Step 1: Initialize sensor via GPIO bit-bang I2C
    if (!bme.begin()) {
        Serial.println("ERROR: Sensor not found!");
        while (1) delay(1000);
    }

    Serial.print("Sensor: ");
    Serial.println(bme.isBME280() ? "BME280" : "BMP280");

    // Step 2: Configure oversampling and filter
    bme.setTemperatureOversampling(BME280_OS_1X);
    bme.setPressureOversampling(BME280_OS_1X);
    bme.setFilter(BME280_FILTER_OFF);

    // Step 3: Load PIO program onto PIO0
    if (!bme.beginPIO(pio0)) {
        Serial.println("ERROR: PIO program load failed!");
        while (1) delay(1000);
    }

    // Step 4: Start continuous DMA auto-scan
    // The sensor is set to NORMAL mode internally.
    // DMA CH1, CH2, and CH3 run in background — no CPU cycles used!
    if (!bme.beginAutoScan(SCAN_PERIOD_MS)) {
        Serial.println("ERROR: DMA auto-scan start failed!");
        while (1) delay(1000);
    }

    Serial.print("Auto-scan started (");
    Serial.print(SCAN_PERIOD_MS);
    Serial.println(" ms period)");
    Serial.println("I2C bit-bang runs in background — CPU free!\n");
}

void loop() {
    float temperature, pressure, humidity;

    // readAllAsync() extracts data from the DMA ring buffer
    // and runs Bosch compensation math (CPU-only, < 1 ms).
    // No I2C communication happens here — the DMA already
    // has the latest data waiting in RAM.
    bme.readAllAsync(&temperature, &pressure, &humidity);

    Serial.print("T: ");
    Serial.print(temperature, 2);
    Serial.print(" °C | P: ");
    Serial.print(pressure, 2);
    Serial.print(" hPa");

    if (bme.isBME280()) {
        Serial.print(" | H: ");
        Serial.print(humidity, 2);
        Serial.print(" %");
    }

    Serial.println();

    // CPU is free to do other work here.
    // The DMA handles all I2C timing in hardware.
    delay(SCAN_PERIOD_MS);
}
