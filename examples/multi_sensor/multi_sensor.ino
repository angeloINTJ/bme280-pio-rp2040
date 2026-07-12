/**
 * @example Multi-Sensor PIO+DMA Example for BMx280PIO_RP2040
 * @brief
 * Reads two BMP280 sensors simultaneously using PIO+DMA burst transfers
 * on independent I2C buses. Each sensor creates its own internal WirePIO
 * instance — zero CPU overhead during I2C reads.
 *
 * PIO Resource Usage:
 *   - PIO block:   pio0 (both sensors share the same program)
 *   - State machines: pio0 SM0 (sensor 1), pio0 SM1 (sensor 2)
 *   - DMA channels:  4 of 12 (2 per WirePIO bus)
 *   - Total: 2 of 8 SMs, 4 of 12 DMA channels
 *
 * This leaves pio1 completely free for other libraries (CYW43 WiFi, etc.).
 *
 * Pico W coexistence:
 *   WiFi uses pio1 (auto-selected by SDK). Both sensors stay on pio0,
 *   leaving pio1 for the CYW43 driver — zero conflict.
 *
 * Wiring:
 *   Sensor 1 VCC → 3.3V    Sensor 2 VCC → 3.3V
 *   Sensor 1 GND → GND      Sensor 2 GND → GND
 *   Sensor 1 SDA → GPIO4    Sensor 2 SDA → GPIO6
 *   Sensor 1 SCL → GPIO5    Sensor 2 SCL → GPIO7
 *
 * Expected output (Serial Monitor, 115200 baud):
 *   BMx280 Multi-Sensor (PIO+DMA)
 *   ==============================
 *   Sensor 1 (GPIO4/5): BMP280 — pio0
 *   Sensor 2 (GPIO6/7): BMP280 — pio0
 *   pio1: completely free
 *   ──────────────────────────────
 *   Sensor 1 | T: 23.45 C | P: 1013.25 hPa | Alt: 0.00 m
 *   Sensor 2 | T: 23.52 C | P: 1013.18 hPa | Alt: 0.06 m
 *     ΔT: -0.07 C  |  ΔP: 0.07 hPa
 */

#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

// ─── Sensor 1: GPIO4 (SDA), GPIO5 (SCL) ─────────────────────────────────
#define SDA1 4
#define SCL1 5

// ─── Sensor 2: GPIO6 (SDA), GPIO7 (SCL) ─────────────────────────────────
#define SDA2 6
#define SCL2 7

// Each sensor creates its own internal WirePIO bus.
// begin() handles GPIO + PIO+DMA initialization automatically.
// Both use pio0 — WirePIO's static cache shares the i2c_master program.
BMx280PIO_RP2040 sensor1(SDA1, SCL1);
BMx280PIO_RP2040 sensor2(SDA2, SCL2);

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\nBMx280 Multi-Sensor (PIO+DMA)");
    Serial.println("==============================");

    // ─── Initialize Sensor 1 ───────────────────────────────────────────
    Serial.print("Sensor 1 (GPIO");
    Serial.print(SDA1);
    Serial.print("/");
    Serial.print(SCL1);
    Serial.print("): ");

    if (!sensor1.begin()) {
        Serial.println("NOT FOUND!");
        Serial.println("  Check: VCC→3.3V, GND→GND, SDA→GPIO4, SCL→GPIO5");
    } else {
        Serial.print(sensor1.isBME280() ? "BME280" : "BMP280");
        Serial.print(" — Chip ID: 0x");
        Serial.print(sensor1.getChipID(), HEX);
        Serial.println(" — pio0 (PIO+DMA via WirePIO)");

        sensor1.setTemperatureOversampling(BME280_OS_2X);
        sensor1.setPressureOversampling(BME280_OS_4X);
        sensor1.setFilter(BME280_FILTER_4);
        sensor1.setMode(BME280_MODE_NORMAL);
    }

    // ─── Initialize Sensor 2 ───────────────────────────────────────────
    Serial.print("Sensor 2 (GPIO");
    Serial.print(SDA2);
    Serial.print("/");
    Serial.print(SCL2);
    Serial.print("): ");

    if (!sensor2.begin()) {
        Serial.println("NOT FOUND!");
        Serial.println("  Check: VCC→3.3V, GND→GND, SDA→GPIO6, SCL→GPIO7");
    } else {
        Serial.print(sensor2.isBME280() ? "BME280" : "BMP280");
        Serial.print(" — Chip ID: 0x");
        Serial.print(sensor2.getChipID(), HEX);
        Serial.println(" — pio0 (PIO+DMA via WirePIO)");

        sensor2.setTemperatureOversampling(BME280_OS_2X);
        sensor2.setPressureOversampling(BME280_OS_4X);
        sensor2.setFilter(BME280_FILTER_4);
        sensor2.setMode(BME280_MODE_NORMAL);
    }

    Serial.println();
    Serial.println("PIO Resource Usage:");
    Serial.println("  pio0: SM0 (sensor 1) + SM1 (sensor 2) — 31 instr shared");
    Serial.println("  pio1: completely free for WirePIO, WiFi, etc.");
    Serial.println("  DMA:  4 of 12 channels used (2 per WirePIO bus)");
    Serial.println("──────────────────────────────");
    Serial.println("Ready.\n");
}

void loop() {
    float t1, p1, h1;
    float t2, p2, h2;

    // ─── Read Sensor 1 (PIO+DMA burst via WirePIO) ────────────────────
    sensor1.readAll(&t1, &p1, &h1);

    // ─── Read Sensor 2 (PIO+DMA burst via WirePIO) ────────────────────
    sensor2.readAll(&t2, &p2, &h2);

    // ─── Output ────────────────────────────────────────────────────────
    Serial.print("Sensor 1 | T: ");
    Serial.print(t1, 2);
    Serial.print(" C | P: ");
    Serial.print(p1, 2);
    Serial.print(" hPa");

    if (!isnan(p1)) {
        float alt1 = 44330.0f * (1.0f - pow(p1 / 1013.25f, 0.1903f));
        Serial.print(" | Alt: ");
        Serial.print(alt1, 2);
        Serial.print(" m");
    }

    if (sensor1.isBME280()) {
        Serial.print(" | H: ");
        Serial.print(h1, 2);
        Serial.print(" %");
    }

    Serial.println();

    Serial.print("Sensor 2 | T: ");
    Serial.print(t2, 2);
    Serial.print(" C | P: ");
    Serial.print(p2, 2);
    Serial.print(" hPa");

    if (!isnan(p2)) {
        float alt2 = 44330.0f * (1.0f - pow(p2 / 1013.25f, 0.1903f));
        Serial.print(" | Alt: ");
        Serial.print(alt2, 2);
        Serial.print(" m");
    }

    if (sensor2.isBME280()) {
        Serial.print(" | H: ");
        Serial.print(h2, 2);
        Serial.print(" %");
    }

    Serial.println();

    // ─── Cross-sensor delta ────────────────────────────────────────────
    if (!isnan(t1) && !isnan(t2)) {
        Serial.print("  DT: ");
        Serial.print(t1 - t2, 2);
        Serial.print(" C  |  DP: ");
        Serial.print(p1 - p2, 2);
        Serial.println(" hPa");
    }

    Serial.println("──────────────────────────────");

    delay(2000);
}
