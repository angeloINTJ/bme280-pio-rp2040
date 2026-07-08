#ifdef BMX280PIO_RP2040_STANDALONE_TEST
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define SDA 2
#define SCL 3
BMx280PIO_RP2040 bme(SDA, SCL);

#define LA_SIZE 4096
static uint32_t la_buf[LA_SIZE];
static volatile uint32_t la_idx=0;
static volatile bool la_go=false, la_done=false;

void core1_la() {
    while(!la_go)tight_loop_contents();
    uint32_t i=0;
    while(la_go&&i<LA_SIZE)la_buf[i++]=sio_hw->gpio_in;
    la_idx=i;la_done=true;
}

void setup() {
    Serial.begin(115200);while(!Serial)delay(100);delay(500);

    // Measure Core 1 sampling rate
    la_go=true; delay(1); la_go=false;
    uint32_t samples_per_ms = la_idx;
    Serial.print("LA rate: ");Serial.print(samples_per_ms);
    Serial.print(" samples/ms (~");Serial.print(1000000/samples_per_ms);
    Serial.println(" ns/sample)");

    // Slow down I2C: use 25kHz instead of 100kHz
    // Re-init PIO with slower clock
    Serial.println("Init...");
    if(!bme.begin()){Serial.println("FAIL");while(1);}
    if(!bme.beginPIO(pio0)){Serial.println("PIO");while(1);}
    multicore_launch_core1(core1_la);
    delay(100);
}

void loop() {
    bme.takeForcedMeasurement();
    la_idx=0;la_done=false;
    delayMicroseconds(100);
    la_go=true;

    float t,p,h;bme.readAll(&t,&p,&h);

    la_go=false;while(!la_done)tight_loop_contents();

    Serial.print("T=");Serial.print(t,2);Serial.print(" P=");Serial.println(p,2);

    // Just print SDA on SCL edges, raw
    uint32_t prev=la_buf[0];
    uint32_t p_scl=(prev>>SCL)&1;
    uint32_t edge_count=0;

    Serial.print("BITS:");
    for(uint32_t i=1;i<la_idx&&edge_count<120;i++){
        uint32_t cur=la_buf[i];
        uint32_t c_scl=(cur>>SCL)&1;
        if(!p_scl && c_scl){ // SCL rising
            Serial.print((cur>>SDA)&1);
            edge_count++;
        }
        p_scl=c_scl;
    }
    Serial.print(" (");Serial.print(edge_count);Serial.println(" edges)");

    // Also print sample count per SCL period (measure actual bit rate)
    Serial.print("SCL periods (samples): ");
    prev=la_buf[0]; p_scl=(prev>>SCL)&1;
    uint32_t last_rise=0, period_count=0;
    for(uint32_t i=1;i<la_idx&&period_count<5;i++){
        uint32_t cur=la_buf[i];
        uint32_t c_scl=(cur>>SCL)&1;
        if(!p_scl && c_scl){
            if(last_rise>0){
                Serial.print(i-last_rise);Serial.print(" ");
                period_count++;
            }
            last_rise=i;
        }
        p_scl=c_scl;
    }
    Serial.println();

    float gt,gp,gh;bme.readAll(&gt,&gp,&gh);
    Serial.print("GPIO T=");Serial.print(gt,2);Serial.print(" P=");Serial.println(gp,2);
    Serial.println();
    delay(2000);
}
#endif
