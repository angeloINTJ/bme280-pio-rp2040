/**
 * WirePIO Transport Deep Diagnostic
 * Tests: GPIO switching, DMA reliability, read-back verification
 */
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

#define SDA 4
#define SCL 5
#define ADDR 0x76

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(2000);

    Serial.println("\n=== WirePIO Transport Diagnostic ===\n");

    BMx280PIO_RP2040 s(SDA, SCL, ADDR, 200000, pio0);
    if (!s.begin()) { Serial.println("begin FAIL!"); return; }
    Serial.print("Chip: "); Serial.println(s.isBME280()?"BME280":"BMP280");

    // Test 1: read chip ID 50x via PIO+DMA, check consistency
    Serial.println("\n--- Test 1: Chip ID consistency (50 reads) ---");
    uint8_t first = 0;
    int mismatches = 0;
    for (int i = 0; i < 50; i++) {
        uint8_t cid = s.readRegister(0xD0);
        if (i == 0) first = cid;
        if (cid != first) {
            mismatches++;
            Serial.print("  MISMATCH at "); Serial.print(i);
            Serial.print(": 0x"); Serial.print(cid, HEX);
            Serial.print(" != 0x"); Serial.println(first, HEX);
        }
        delayMicroseconds(500);
    }
    Serial.print("  First: 0x"); Serial.print(first, HEX);
    Serial.print("  Mismatches: "); Serial.println(mismatches);

    // Test 2: read calibration 10x, compare
    Serial.println("\n--- Test 2: Calibration consistency (10 reads) ---");
    uint8_t cal0[6], cal[6];
    s.readRegisters(0x88, cal0, 6);
    int calMismatches = 0;
    for (int r = 1; r < 10; r++) {
        s.readRegisters(0x88, cal, 6);
        for (int i = 0; i < 6; i++) {
            if (cal[i] != cal0[i]) {
                calMismatches++;
                Serial.print("  CAL["); Serial.print(i);
                Serial.print("] mismatch at read "); Serial.print(r);
                Serial.print(": 0x"); Serial.print(cal[i], HEX);
                Serial.print(" != 0x"); Serial.println(cal0[i], HEX);
            }
        }
        delayMicroseconds(500);
    }
    Serial.print("  Calibration mismatches: "); Serial.println(calMismatches);

    // Test 3: setMode + verify CTRL_MEAS register
    Serial.println("\n--- Test 3: setMode verification ---");
    uint8_t ctrl_before = s.readRegister(0xF4);
    Serial.print("  CTRL_MEAS before setMode(NORMAL): 0x");
    Serial.println(ctrl_before, HEX);

    s.setMode(BME280_MODE_NORMAL);
    delay(50);

    uint8_t ctrl_after = s.readRegister(0xF4);
    Serial.print("  CTRL_MEAS after setMode(NORMAL):  0x");
    Serial.println(ctrl_after, HEX);

    // Verify: should have osrs_t=1 (bits 7:5 = 001), osrs_p=1 (bits 4:2 = 001), mode=NORMAL (bits 1:0 = 11)
    // Expected: 0b00100111 = 0x27
    uint8_t expected = 0x27;
    bool ctrl_ok = (ctrl_after == expected);
    Serial.print("  Expected: 0x27  "); Serial.println(ctrl_ok ? "OK" : "FAIL!");

    // Test 4: Sensor data 5x after NORMAL mode
    Serial.println("\n--- Test 4: Sensor readings in NORMAL mode ---");
    s.setMode(BME280_MODE_NORMAL);
    delay(200); // wait for first measurement
    for (int i = 0; i < 5; i++) {
        float t = s.readTemperature();
        float p = s.readPressure();
        uint8_t d[8];
        s.readRegisters(0xF7, d, 8);
        Serial.print("  ["); Serial.print(i); Serial.print("] raw=[");
        for (int j=0;j<8;j++){Serial.print(d[j],HEX);Serial.print(" ");}
        Serial.print("] T="); Serial.print(t,1);
        Serial.print("C P="); Serial.print(p,0);
        Serial.println("hPa");
        delay(250);
    }

    // Test 5: Compare with force GPIO on same sensor
    Serial.println("\n--- Test 5: GPIO bit-bang comparison (same sensor) ---");
    {
        BMx280PIO_RP2040 s2(SDA, SCL, ADDR);
        s2.forceGPIO(true);
        s2.begin();
        s2.setMode(BME280_MODE_NORMAL);
        delay(200);
        float t, p;
        s2.readAll(&t, &p, nullptr);
        uint8_t d[8];
        s2.readRegisters(0xF7, d, 8);
        Serial.print("  GPIO: raw=[");
        for (int j=0;j<8;j++){Serial.print(d[j],HEX);Serial.print(" ");}
        Serial.print("] T="); Serial.print(t,1);
        Serial.print("C P="); Serial.print(p,0);
        Serial.println("hPa");
    }

    Serial.println("\n=== Done ===");
}

void loop() { delay(1000); }
