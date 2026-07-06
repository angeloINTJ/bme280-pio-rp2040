/*
 * PIO_I2C.cpp - I2C Master for RP2040 (GPIO bit-bang + PIO-ready)
 *
 * Transport layer for I2C communication on any GPIO pins.
 * Uses software GPIO bit-bang with open-drain SDA emulation.
 *
 * PIO-READY: The PIO assembly program (pio/i2c.pio) implements the full
 * I2C master protocol in hardware. It requires OUT-based SDA release
 * (not SET-based, due to RP2040 OUT priority over SET).
 * Current limitation: SDA release via out pindirs after START is unreliable
 * on some RP2040 silicon revisions. Software fallback used until resolved.
 *
 * Architecture follows OneWirePIO_RP2040 / DHT22PIO_RP2040 pattern.
 */

#include "PIO_I2C.h"
#include <hardware/gpio.h>

PIO_I2C::PIO_I2C(uint8_t sda, uint8_t scl, uint32_t freq)
    : _sda(sda), _scl(scl), _freq(freq), _pio(nullptr), _sm(0), _offset(0), _initialized(false) {}

bool PIO_I2C::begin() {
    if (_initialized) return true;
    gpio_init(_sda); gpio_set_dir(_sda, GPIO_IN); gpio_pull_up(_sda);
    gpio_init(_scl); gpio_set_dir(_scl, GPIO_OUT); gpio_put(_scl, 1);
    _initialized = true;
    return true;
}
void PIO_I2C::end() {
    if (!_initialized) return;
    gpio_set_dir(_sda, GPIO_IN); gpio_disable_pulls(_sda);
    gpio_set_dir(_scl, GPIO_IN); gpio_disable_pulls(_scl);
    _initialized = false;
}

// GPIO helpers
static inline void sda_lo(uint p) { gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0); }
static inline void sda_hi(uint p) { gpio_set_dir(p, GPIO_IN); }
static inline bool sda_read(uint p) { return gpio_get(p); }
static inline void scl_lo(uint p) { gpio_put(p, 0); }
static inline void scl_hi(uint p) { gpio_put(p, 1); }
static void i2c_delay(uint32_t f) { delayMicroseconds(500000/f < 2 ? 2 : 500000/f); }

// I2C primitives
static bool i2c_write_byte(uint sd, uint sc, uint32_t f, uint8_t d) {
    for (uint8_t m=0x80;m;m>>=1){if(d&m)sda_hi(sd);else sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);scl_lo(sc);}
    sda_hi(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);bool n=sda_read(sd);scl_lo(sc);i2c_delay(f);return !n;
}
static uint8_t i2c_read_byte(uint sd, uint sc, uint32_t f, bool last) {
    uint8_t d=0;sda_hi(sd);
    for(int i=0;i<8;i++){scl_hi(sc);i2c_delay(f);d=(d<<1)|(sda_read(sd)?1:0);scl_lo(sc);i2c_delay(f);}
    if(last)sda_hi(sd);else sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);scl_lo(sc);i2c_delay(f);sda_hi(sd);return d;
}
static void i2c_start(uint sd,uint sc,uint32_t f){sda_lo(sd);i2c_delay(f);scl_lo(sc);i2c_delay(f);}
static void i2c_stop(uint sd,uint sc,uint32_t f){sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);sda_hi(sd);i2c_delay(f);}

// Public API
bool PIO_I2C::_sendAddress(uint8_t a,bool r){i2c_start(_sda,_scl,_freq);return i2c_write_byte(_sda,_scl,_freq,(a<<1)|(r?1:0));}
void PIO_I2C::_sendStop(){i2c_stop(_sda,_scl,_freq);}
void PIO_I2C::_waitIdle(){}
void PIO_I2C::putCommand(uint16_t){}
bool PIO_I2C::hasData(){return false;}
uint32_t PIO_I2C::readFIFO(){return 0;}
void PIO_I2C::clearFIFO(){}

bool PIO_I2C::write(uint8_t a,const uint8_t*d,size_t n,bool s){
    if(!_initialized||n==0)return false;
    i2c_start(_sda,_scl,_freq);
    if(!i2c_write_byte(_sda,_scl,_freq,(a<<1))){i2c_stop(_sda,_scl,_freq);return false;}
    for(size_t i=0;i<n;i++){if(!i2c_write_byte(_sda,_scl,_freq,d[i])){i2c_stop(_sda,_scl,_freq);return false;}}
    if(s)i2c_stop(_sda,_scl,_freq);return true;
}
bool PIO_I2C::read(uint8_t a,uint8_t*d,size_t n,bool s){
    if(!_initialized||n==0)return false;
    i2c_start(_sda,_scl,_freq);
    if(!i2c_write_byte(_sda,_scl,_freq,(a<<1)|1)){i2c_stop(_sda,_scl,_freq);return false;}
    for(size_t i=0;i<n;i++)d[i]=i2c_read_byte(_sda,_scl,_freq,(i==n-1));
    if(s)i2c_stop(_sda,_scl,_freq);return true;
}
bool PIO_I2C::writeThenRead(uint8_t a,const uint8_t*wd,size_t wl,uint8_t*rd,size_t rl){
    if(!_initialized)return false;
    if(wl>0){i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1))){i2c_stop(_sda,_scl,_freq);return false;}for(size_t i=0;i<wl;i++){if(!i2c_write_byte(_sda,_scl,_freq,wd[i])){i2c_stop(_sda,_scl,_freq);return false;}}i2c_stop(_sda,_scl,_freq);}
    if(rl>0){i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1)|1)){i2c_stop(_sda,_scl,_freq);return false;}for(size_t i=0;i<rl;i++)rd[i]=i2c_read_byte(_sda,_scl,_freq,(i==rl-1));i2c_stop(_sda,_scl,_freq);}
    return true;
}
void PIO_I2C::scan(){
    if(!_initialized)return;Serial.println("I2C Scan:");int f=0;
    for(int a=1;a<0x78;a++){i2c_start(_sda,_scl,_freq);bool ack=i2c_write_byte(_sda,_scl,_freq,(a<<1));i2c_stop(_sda,_scl,_freq);if(ack){Serial.print("0x");Serial.print(a,HEX);Serial.print(" ");f++;}}
    Serial.print("(");Serial.print(f);Serial.println(" devices)");
}
