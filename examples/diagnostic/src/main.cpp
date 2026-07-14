/**
 * Overnight Stability Test — 5x BMP280 PIO+DMA continuous loop
 * Reports failures immediately, prints hourly summary
 */
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

#define ADDR 0x76
struct { const char *n; uint8_t sda, scl; PIO p; } cfg[]={
    {"S0",0,1,pio0},{"S1",2,3,pio0},{"S2",4,5,pio0},
    {"S3",6,7,pio1},{"S4",8,9,pio1},
};
#define N 5

// Stats
uint32_t g_total=0, g_ok=0, g_fail_init=0, g_fail_read=0;
uint32_t g_last_print=0;
float    g_tmin[N], g_tmax[N], g_pmin[N], g_pmax[N];

void setup(){
    Serial.begin(115200);while(!Serial)delay(10);delay(2000);
    Serial.println("\n=== Overnight Stability Test ===\n");
    Serial.println("Sensors: S0(0/1) S1(2/3) S2(4/5) S3(6/7) S4(8/9)");
    Serial.println("Mode: PIO+DMA @ 200 kHz | Reporting failures + hourly summary\n");

    for(int i=0;i<N;i++){g_tmin[i]=999;g_tmax[i]=-999;g_pmin[i]=999999;g_pmax[i]=-999999;}

    // Init all sensors once
    BMx280PIO_RP2040 *dev[N]={0};

    for(int i=0;i<N;i++){
        dev[i]=new BMx280PIO_RP2040(cfg[i].sda,cfg[i].scl,ADDR,200000,cfg[i].p);
        if(!dev[i]->begin()){
            Serial.print("INIT FAIL: ");Serial.println(cfg[i].n);
            delete dev[i];dev[i]=0;g_fail_init++;
            continue;
        }
        dev[i]->setMode(BME280_MODE_NORMAL);
        Serial.print(cfg[i].n);Serial.print(" init OK ");
    }
    Serial.println();
    delay(300); // let first measurement complete
    g_last_print=millis();

    // ─── Infinite loop ──────────────────────────────────────────
    for (uint32_t iter=1;;iter++){
        bool any_fail=false;

        Serial.print("[");Serial.print(iter);Serial.print("] ");
        for(int i=0;i<N;i++){
            if(!dev[i]){Serial.print("---/--- ");continue;}

            uint32_t t0=micros();
            float t,p;
            dev[i]->readAll(&t,&p,nullptr);
            uint32_t dt=micros()-t0;
            g_total++;

            bool ok=(!isnan(t)&&!isnan(p)&&t>-40&&t<85&&p>300&&p<2000);
            if(ok){
                g_ok++;
                if(t<g_tmin[i])g_tmin[i]=t;if(t>g_tmax[i])g_tmax[i]=t;
                if(p<g_pmin[i])g_pmin[i]=p;if(p>g_pmax[i])g_pmax[i]=p;
                Serial.print(t,1);Serial.print("C/");Serial.print(p,0);Serial.print("hPa ");
            } else {
                g_fail_read++;
                any_fail=true;
                Serial.print("FAIL(");Serial.print(t,1);Serial.print("/");Serial.print(p,0);Serial.print(") ");
            }
        }
        Serial.println();

        // Hourly summary (every ~3600s) — printed between readings
        uint32_t now=millis();
        if(now-g_last_print>=3600000){
            g_last_print=now;
            float hours=now/3600000.0f;
            Serial.println();
            Serial.println("========== HOURLY SUMMARY ==========");
            Serial.println("Method: PIO+DMA (WirePIO) @ 200 kHz");
            Serial.print("Uptime: ");Serial.print(hours,1);Serial.println("h");
            Serial.print("Reads: ");Serial.print(g_total);
            Serial.print("  OK: ");Serial.print(g_ok);
            Serial.print("  Fail: ");Serial.print(g_fail_read);
            Serial.print("  Success: ");Serial.print(g_total>0?(100.0f*g_ok/g_total):0,1);
            Serial.println("%");
            Serial.println("Sensor   Tmin   Tmax   Pmin   Pmax");
            for(int i=0;i<N;i++){
                Serial.print(cfg[i].n);Serial.print("       ");
                if(g_tmin[i]<900){Serial.print(g_tmin[i],1);Serial.print("  ");Serial.print(g_tmax[i],1);}
                else Serial.print("  ---   --- ");
                Serial.print("  ");
                if(g_pmin[i]<900000){Serial.print(g_pmin[i],0);Serial.print(" ");Serial.print(g_pmax[i],0);}
                else Serial.print(" ---  ---");
                Serial.println();
            }
            Serial.println("=====================================\n");
        }

        delay(100); // ~10 readings/sec/sensor
    }
}

void loop(){}
