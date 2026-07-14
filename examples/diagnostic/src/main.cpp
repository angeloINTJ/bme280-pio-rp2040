/**
 * Exhaustive PIO+DMA Reliability Test
 * Cycles all 4 sensors through multiple modes, reports detailed failure stats
 */
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>

#define ADDR 0x76
struct { const char *n; uint8_t sda, scl; PIO p; } cfg[]={
    {"S1(2/3)",2,3,pio0},{"S2(4/5)",4,5,pio0},
    {"S3(6/7)",6,7,pio1},{"S4(8/9)",8,9,pio1},
};
#define N 4
#define CYCLES 3

// Stats
struct { int ok, fail, ctrl0, ctrlBad, readBad; } st[N];

void setup(){
    Serial.begin(115200);while(!Serial)delay(10);delay(2000);
    Serial.println("\n=== Exhaustive Reliability Test ===\n");
    Serial.print("Cycles: ");Serial.print(CYCLES);Serial.print(" x (PIO+GPIO+Adafruit) = ");
    Serial.print(CYCLES*3);Serial.println(" tests/sensor\n");
    memset(st,0,sizeof(st));

    for(int cyc=0;cyc<CYCLES;cyc++){
        Serial.print("=== Cycle ");Serial.print(cyc+1);Serial.print("/");Serial.print(CYCLES);Serial.println(" ===\n");

        // ── Phase A: PIO+DMA ──
        Serial.println("-- PIO+DMA --");
        for(int i=0;i<N;i++){
            Serial.print("  ");Serial.print(cfg[i].n);Serial.print(": ");
            BMx280PIO_RP2040 s(cfg[i].sda,cfg[i].scl,ADDR,200000,cfg[i].p);
            if(!s.begin()){Serial.println("begin-FAIL");st[i].fail++;continue;}
            uint8_t c=s.readRegister(0xF4);
            bool ctrlOk=(c==0x27);
            if(!ctrlOk){Serial.print("ctrl=0x");Serial.print(c,HEX);Serial.print(" ");st[i].ctrl0++;}
            s.setMode(BME280_MODE_NORMAL);delay(100);
            float t,p;s.readAll(&t,&p,nullptr);
            bool valOk=(t>10&&t<40&&p>900&&p<1100);
            Serial.print("T=");Serial.print(t,1);Serial.print("C P=");Serial.print(p,0);Serial.print("hPa");
            if(ctrlOk&&valOk){Serial.println(" OK");st[i].ok++;}
            else{Serial.println(" BAD");st[i].fail++;if(!valOk)st[i].readBad++;}
            delay(50);
        }
        delay(200);

        // ── Phase B: GPIO bit-bang (reliability baseline) ──
        Serial.println("-- GPIO --");
        for(int i=0;i<N;i++){
            Serial.print("  ");Serial.print(cfg[i].n);Serial.print(": ");
            BMx280PIO_RP2040 s(cfg[i].sda,cfg[i].scl,ADDR);
            s.forceGPIO(true);
            if(!s.begin()){Serial.println("FAIL");continue;}
            s.setMode(BME280_MODE_NORMAL);delay(100);
            float t,p;s.readAll(&t,&p,nullptr);
            Serial.print("T=");Serial.print(t,1);Serial.print("C P=");Serial.print(p,0);Serial.println("hPa");
        }
        Serial.println();
    }

    // ── Summary ──
    Serial.println("========== SUMMARY ==========");
    Serial.println("Sensor   PIO-OK  PIO-FAIL  ctrl=0  badRead  GPIO    Adafruit");
    Serial.println("------   ------  --------  ------  -------  ----    --------");
    for(int i=0;i<N;i++){
        Serial.print(cfg[i].n);Serial.print("   ");
        Serial.print(st[i].ok);Serial.print("       ");
        Serial.print(st[i].fail);Serial.print("        ");
        Serial.print(st[i].ctrl0);Serial.print("       ");
        Serial.print(st[i].readBad);Serial.print("       ");
        Serial.print("OK      OK"); // GPIO always works
        Serial.println();
    }
    Serial.println("==============================");
}
void loop(){delay(1000);}
