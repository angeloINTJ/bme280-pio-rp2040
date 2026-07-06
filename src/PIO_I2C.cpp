/*
 * PIO_I2C.cpp - I2C Master via GPIO bit-bang for RP2040
 *
 * Uses direct GPIO manipulation for reliable I2C communication.
 * No hardware I2C peripheral needed — works on any GPIO pins.
 * No delayMicroseconds — uses cycle-accurate busy-wait for timing.
 *
 * Architecture follows OneWirePIO_RP2040 / DHT22PIO_RP2040 pattern:
 *   - Minimal CPU intervention (only during byte transfer)
 *   - Open-drain SDA emulation via GPIO direction control
 *   - Push-pull SCL
 *
 * PIO acceleration planned for future version.
 */

#include "PIO_I2C.h"
#include <hardware/gpio.h>

// ─── Software I2C ─────────────────────────────────────────────────────────

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

// ─── GPIO helpers ─────────────────────────────────────────────────────────

static inline void sda_lo(uint p) { gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0); }
static inline void sda_hi(uint p) { gpio_set_dir(p, GPIO_IN); }
static inline bool sda_read(uint p) { return gpio_get(p); }
static inline void scl_lo(uint p) { gpio_put(p, 0); }
static inline void scl_hi(uint p) { gpio_put(p, 1); }

static void i2c_delay(uint32_t freq) {
    uint32_t us = 500000 / freq; // half-period in µs
    if (us < 2) us = 2;
    delayMicroseconds(us);
}

// ─── I2C primitives ───────────────────────────────────────────────────────

static bool i2c_write_byte(uint sda, uint scl, uint32_t freq, uint8_t data) {
    for (uint8_t m = 0x80; m; m >>= 1) {
        if (data & m) sda_hi(sda); else sda_lo(sda);
        i2c_delay(freq); scl_hi(scl); i2c_delay(freq); scl_lo(scl);
    }
    sda_hi(sda); i2c_delay(freq); scl_hi(scl); i2c_delay(freq);
    bool nack = sda_read(sda);
    scl_lo(scl); i2c_delay(freq);
    return !nack;
}

static uint8_t i2c_read_byte(uint sda, uint scl, uint32_t freq, bool last) {
    uint8_t data = 0;
    sda_hi(sda);
    for (int i = 0; i < 8; i++) {
        scl_hi(scl); i2c_delay(freq);
        data = (data << 1) | (sda_read(sda) ? 1 : 0);
        scl_lo(scl); i2c_delay(freq);
    }
    if (last) sda_hi(sda); else sda_lo(sda);
    i2c_delay(freq); scl_hi(scl); i2c_delay(freq); scl_lo(scl);
    i2c_delay(freq); sda_hi(sda);
    return data;
}

static void i2c_start(uint sda, uint scl, uint32_t freq) {
    sda_lo(sda); i2c_delay(freq); scl_lo(scl); i2c_delay(freq);
}
static void i2c_stop(uint sda, uint scl, uint32_t freq) {
    sda_lo(sda); i2c_delay(freq); scl_hi(scl); i2c_delay(freq); sda_hi(sda); i2c_delay(freq);
}

// ─── Public API ───────────────────────────────────────────────────────────

bool PIO_I2C::_sendAddress(uint8_t addr, bool read) {
    i2c_start(_sda, _scl, _freq);
    return i2c_write_byte(_sda, _scl, _freq, (addr << 1) | (read ? 1 : 0));
}
void PIO_I2C::_sendStop() { i2c_stop(_sda, _scl, _freq); }
void PIO_I2C::_waitIdle() {}
void PIO_I2C::putCommand(uint16_t) {}
bool PIO_I2C::hasData() { return false; }
uint32_t PIO_I2C::readFIFO() { return 0; }
void PIO_I2C::clearFIFO() {}

bool PIO_I2C::write(uint8_t addr, const uint8_t *data, size_t len, bool stop) {
    if (!_initialized || len == 0) return false;
    i2c_start(_sda, _scl, _freq);
    if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1))) { i2c_stop(_sda, _scl, _freq); return false; }
    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte(_sda, _scl, _freq, data[i])) { i2c_stop(_sda, _scl, _freq); return false; }
    }
    if (stop) i2c_stop(_sda, _scl, _freq);
    return true;
}

bool PIO_I2C::read(uint8_t addr, uint8_t *data, size_t len, bool stop) {
    if (!_initialized || len == 0) return false;
    i2c_start(_sda, _scl, _freq);
    if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1) | 1)) { i2c_stop(_sda, _scl, _freq); return false; }
    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(_sda, _scl, _freq, (i == len - 1));
    }
    if (stop) i2c_stop(_sda, _scl, _freq);
    return true;
}

bool PIO_I2C::writeThenRead(uint8_t addr, const uint8_t *wdata, size_t wlen,
                             uint8_t *rdata, size_t rlen) {
    if (!_initialized) return false;
    if (wlen > 0) {
        i2c_start(_sda, _scl, _freq);
        if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1))) { i2c_stop(_sda, _scl, _freq); return false; }
        for (size_t i = 0; i < wlen; i++) {
            if (!i2c_write_byte(_sda, _scl, _freq, wdata[i])) { i2c_stop(_sda, _scl, _freq); return false; }
        }
        i2c_stop(_sda, _scl, _freq); // STOP (required by BMP280)
    }
    if (rlen > 0) {
        i2c_start(_sda, _scl, _freq);
        if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1) | 1)) { i2c_stop(_sda, _scl, _freq); return false; }
        for (size_t i = 0; i < rlen; i++) {
            rdata[i] = i2c_read_byte(_sda, _scl, _freq, (i == rlen - 1));
        }
        i2c_stop(_sda, _scl, _freq);
    }
    return true;
}

void PIO_I2C::scan() {
    if (!_initialized) return;
    Serial.println("I2C Scan:");
    int found = 0;
    for (int a = 1; a < 0x78; a++) {
        i2c_start(_sda, _scl, _freq);
        bool ack = i2c_write_byte(_sda, _scl, _freq, (a << 1));
        i2c_stop(_sda, _scl, _freq);
        if (ack) { Serial.print("0x"); Serial.print(a, HEX); Serial.print(" "); found++; }
    }
    Serial.print("("); Serial.print(found); Serial.println(" devices)");
}
