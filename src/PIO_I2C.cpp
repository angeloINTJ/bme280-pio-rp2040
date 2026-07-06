/*
 * PIO_I2C.cpp - I2C Master via software GPIO bit-bang for RP2040
 *
 * Uses direct GPIO manipulation for reliable I2C communication.
 * Standard mode (100 kHz) and Fast mode (400 kHz) supported.
 * Works on any pair of GPIO pins.
 *
 * The PIO name is kept for API compatibility. A future version
 * will add true PIO state machine acceleration for higher speeds.
 */

#include "PIO_I2C.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>

// ─── Software I2C Implementation ──────────────────────────────────────────

PIO_I2C::PIO_I2C(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq)
    : _sda(sda_pin), _scl(scl_pin), _freq(freq)
    , _pio(nullptr), _sm(0), _offset(0), _initialized(false)
{
}

bool PIO_I2C::begin() {
    if (_initialized) return true;

    // Configure SDA: open-drain emulation
    gpio_init(_sda);
    gpio_set_dir(_sda, GPIO_IN);
    gpio_pull_up(_sda);

    // Configure SCL: push-pull output
    gpio_init(_scl);
    gpio_set_dir(_scl, GPIO_OUT);
    gpio_put(_scl, 1); // idle high

    _initialized = true;
    return true;
}

void PIO_I2C::end() {
    if (!_initialized) return;
    gpio_set_dir(_sda, GPIO_IN);
    gpio_disable_pulls(_sda);
    gpio_set_dir(_scl, GPIO_IN);
    gpio_disable_pulls(_scl);
    _initialized = false;
}

// ─── GPIO helpers ─────────────────────────────────────────────────────────

// SDA control: 0 = drive low (output), 1 = release (input with pull-up)
static inline void i2c_sda_out(uint pin)  { gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0); }
static inline void i2c_sda_in(uint pin)   { gpio_set_dir(pin, GPIO_IN); }
static inline bool i2c_sda_read(uint pin) { return gpio_get(pin); }

static inline void i2c_scl_lo(uint pin) { gpio_put(pin, 0); }
static inline void i2c_scl_hi(uint pin) { gpio_put(pin, 1); }

// Half-period delay for the configured frequency
static inline void i2c_delay_half(uint32_t freq) {
    // Use busy-wait for accurate short delays
    uint32_t us = 500000 / freq;
    if (us < 2) us = 2;
    // busy_wait is more accurate than delayMicroseconds for short delays
    delayMicroseconds(us);
}

// ─── I2C Primitives ──────────────────────────────────────────────────────

static void i2c_start(uint sda, uint scl, uint32_t freq) {
    i2c_sda_out(sda);
    i2c_delay_half(freq);
    i2c_scl_lo(scl);
    i2c_delay_half(freq);
}

static void i2c_stop(uint sda, uint scl, uint32_t freq) {
    i2c_sda_out(sda);
    i2c_delay_half(freq);
    i2c_scl_hi(scl);
    i2c_delay_half(freq);
    i2c_sda_in(sda);
    i2c_delay_half(freq);
}

static bool i2c_write_byte(uint sda, uint scl, uint32_t freq, uint8_t data) {
    // Send 8 bits, MSB first
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        if (data & mask) i2c_sda_in(sda); else i2c_sda_out(sda);
        i2c_delay_half(freq);
        i2c_scl_hi(scl);
        i2c_delay_half(freq);
        i2c_scl_lo(scl);
    }

    // Release SDA for ACK
    i2c_sda_in(sda);
    i2c_delay_half(freq);
    i2c_scl_hi(scl);
    i2c_delay_half(freq);

    // Read ACK (low = ACK, high = NACK)
    bool nack = i2c_sda_read(sda);
    i2c_scl_lo(scl);
    i2c_delay_half(freq);

    return !nack; // true = ACK received
}

static uint8_t i2c_read_byte(uint sda, uint scl, uint32_t freq, bool last) {
    uint8_t data = 0;

    // Release SDA so slave can drive it
    i2c_sda_in(sda);

    // Read 8 bits, MSB first
    for (int i = 0; i < 8; i++) {
        i2c_scl_hi(scl);
        i2c_delay_half(freq);
        data = (data << 1) | (i2c_sda_read(sda) ? 1 : 0);
        i2c_scl_lo(scl);
        i2c_delay_half(freq);
    }

    // Send ACK (0) or NACK (1)
    if (last) i2c_sda_in(sda);  // NACK: release SDA
    else      i2c_sda_out(sda); // ACK: drive low

    i2c_delay_half(freq);
    i2c_scl_hi(scl);
    i2c_delay_half(freq);
    i2c_scl_lo(scl);
    i2c_delay_half(freq);

    // Release SDA
    i2c_sda_in(sda);

    return data;
}

// ─── Public API ───────────────────────────────────────────────────────────

bool PIO_I2C::_sendAddress(uint8_t addr, bool read) {
    i2c_start(_sda, _scl, _freq);
    uint8_t ab = (addr << 1) | (read ? 1 : 0);
    return i2c_write_byte(_sda, _scl, _freq, ab);
}

void PIO_I2C::_sendStop() {
    i2c_stop(_sda, _scl, _freq);
}

void PIO_I2C::_waitIdle() {
    delayMicroseconds(2);
}

bool PIO_I2C::write(uint8_t addr, const uint8_t *data, size_t len, bool stop) {
    if (!_initialized || len == 0) return false;

    i2c_start(_sda, _scl, _freq);
    if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1))) {
        i2c_stop(_sda, _scl, _freq);
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte(_sda, _scl, _freq, data[i])) {
            i2c_stop(_sda, _scl, _freq);
            return false;
        }
    }

    if (stop) i2c_stop(_sda, _scl, _freq);
    return true;
}

bool PIO_I2C::read(uint8_t addr, uint8_t *data, size_t len, bool stop) {
    if (!_initialized || len == 0) return false;

    i2c_start(_sda, _scl, _freq);
    if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1) | 1)) {
        i2c_stop(_sda, _scl, _freq);
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(_sda, _scl, _freq, (i == len - 1));
    }

    if (stop) i2c_stop(_sda, _scl, _freq);
    return true;
}

bool PIO_I2C::writeThenRead(uint8_t addr,
                             const uint8_t *wdata, size_t wlen,
                             uint8_t *rdata, size_t rlen) {
    if (!_initialized) return false;

    // Write phase: write register address, then STOP
    // BMP280 needs a STOP before reading (doesn't support repeated START reliably)
    if (wlen > 0) {
        i2c_start(_sda, _scl, _freq);
        if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1))) {
            i2c_stop(_sda, _scl, _freq);
            return false;
        }
        for (size_t i = 0; i < wlen; i++) {
            if (!i2c_write_byte(_sda, _scl, _freq, wdata[i])) {
                i2c_stop(_sda, _scl, _freq);
                return false;
            }
        }
        i2c_stop(_sda, _scl, _freq); // STOP after write
    }

    // Read phase: START + addr+R + read data
    if (rlen > 0) {
        delayMicroseconds(50);
        i2c_start(_sda, _scl, _freq);
        if (!i2c_write_byte(_sda, _scl, _freq, (addr << 1) | 1)) {
            i2c_stop(_sda, _scl, _freq);
            return false;
        }
        for (size_t i = 0; i < rlen; i++) {
            rdata[i] = i2c_read_byte(_sda, _scl, _freq, (i == rlen - 1));
        }
        i2c_stop(_sda, _scl, _freq);
    }

    return true;
}

void PIO_I2C::scan() {
    if (!_initialized) return;

    Serial.println("I2C Bus Scan (GPIO bit-bang):");
    int found = 0;
    for (int a = 1; a < 0x78; a++) {
        i2c_start(_sda, _scl, _freq);
        bool ack = i2c_write_byte(_sda, _scl, _freq, (a << 1));
        i2c_stop(_sda, _scl, _freq);

        if (ack) {
            Serial.print("0x"); Serial.print(a, HEX); Serial.print(" ");
            found++;
        }
        delayMicroseconds(50);
    }
    Serial.print("("); Serial.print(found); Serial.println(" devices)");
}
