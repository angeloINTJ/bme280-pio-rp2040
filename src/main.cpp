/*
 * BMx280_PIO Library Test — using the library with hardware I2C (Wire1)
 * Auto-detects BMP280 vs BME280
 * GP2=SDA, GP3=SCL on RP2040 (I2C1)
 */

#include <Arduino.h>
#include <Wire.h>
#include "BMx280_PIO.h"

BMx280_PIO *sensor = nullptr;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    delay(500);

    Serial.println("=== BMx280_PIO Library Test ===\n");

    // Init hardware I2C
    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire1.setTimeout(5000);
    Wire1.begin();
    Serial.println("I2C: Wire1 on GP2(SDA)/GP3(SCL)");

    // Create sensor
    sensor = new BMx280_PIO(Wire1, 0x76);

    Serial.print("Sensor begin()... ");
    if (!sensor->begin()) {
        Serial.println("FAILED!");
        Serial.println("Check: VCC=3.3V, GND, SDA=GP2, SCL=GP3, pull-ups=10k");
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
        }
    }
    Serial.println("OK");

    // Report sensor type
    Serial.print("Chip ID: 0x");
    Serial.print(sensor->getChipID(), HEX);
    Serial.print(" (");
    Serial.print(sensor->isBME280() ? "BME280" : "BMP280");
    Serial.println(")\n");

    // Configure: 1x oversampling, forced mode
    sensor->setTemperatureOversampling(BME280_OS_1X);
    sensor->setPressureOversampling(BME280_OS_1X);
    if (sensor->isBME280()) sensor->setHumidityOversampling(BME280_OS_1X);
    sensor->setFilter(BME280_FILTER_OFF);

    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("--- Readings (1/sec, 20 samples) ---");
    Serial.println("  #    Temp(C)   Press(hPa)   Hum(%)");
}

void loop() {
    static int n = 0;
    static float t_sum = 0, p_sum = 0, t_min = 999, t_max = -999, p_min = 9999, p_max = 0;

    if (n >= 20) {
        Serial.println("\n--- Summary (20 samples) ---");
        Serial.print("Chip: ");
        Serial.println(sensor->isBME280() ? "BME280" : "BMP280");
        Serial.print("T: avg="); Serial.print(t_sum / 20.0f, 2);
        Serial.print(" min="); Serial.print(t_min, 2);
        Serial.print(" max="); Serial.print(t_max, 2);
        Serial.println(" C");
        Serial.print("P: avg="); Serial.print(p_sum / 20.0f, 2);
        Serial.print(" min="); Serial.print(p_min, 2);
        Serial.print(" max="); Serial.print(p_max, 2);
        Serial.println(" hPa");
        Serial.println("\n=== TEST PASSED ===");

        while (1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW);  delay(800);
        }
    }

    // Take forced measurement
    sensor->takeForcedMeasurement();

    // Read all values in one burst
    float t, p, h;
    sensor->readAll(&t, &p, &h);

    Serial.print("  "); Serial.print(n); Serial.print("    ");
    Serial.print(t, 2); Serial.print("      ");
    Serial.print(p, 2); Serial.print("       ");
    Serial.println(h, 2);

    t_sum += t; p_sum += p;
    if (t < t_min) t_min = t; if (t > t_max) t_max = t;
    if (p < p_min) p_min = p; if (p > p_max) p_max = p;
    n++;

    digitalWrite(LED_BUILTIN, HIGH); delay(20);
    digitalWrite(LED_BUILTIN, LOW);  delay(980);
}
