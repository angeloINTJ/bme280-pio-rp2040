/**
 * @file standalone_test.cpp
 * @brief PlatformIO standalone test — compiled with -DBMX280PIO_RP2040_STANDALONE_TEST.
 *
 * This file is only compiled when building the project directly via PlatformIO.
 * When installed as an Arduino library, the flag is absent so no conflicting
 * setup()/loop() symbols are generated.
 */
#ifdef BMX280PIO_RP2040_STANDALONE_TEST
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

BMx280PIO_RP2040 bme(2, 3);

void setup() {
    Serial.begin(115200); while (!Serial) delay(100); delay(500);
    Serial.println("BMx280PIO_RP2040 Test");
    if (!bme.begin()) { Serial.println("Sensor fail!"); while (1); }
    Serial.print("Sensor: "); Serial.println(bme.isBME280() ? "BME280" : "BMP280");
}

void loop() {
    bme.takeForcedMeasurement();
    float t, p, h; bme.readAll(&t, &p, &h);
    Serial.print("T="); Serial.print(t, 2); Serial.print("C P="); Serial.print(p, 2); Serial.println("hPa");
    delay(2000);
}
#endif
