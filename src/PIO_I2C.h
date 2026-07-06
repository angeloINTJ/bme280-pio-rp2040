/*
 * PIO_I2C.h - I2C Master via PIO state machine for RP2040
 *
 * Architecture: same pattern as OneWirePIO_RP2040 / DHT22PIO_RP2040
 *   - PIO handles ALL I2C timing (no CPU delays in transport layer)
 *   - FIFO-based command/response interface
 *   - Dynamic PIO allocation (pio0 or pio1)
 *   - RAII: SM released on destruction
 */

#ifndef PIO_I2C_H
#define PIO_I2C_H

#include <Arduino.h>

#define PIO_I2C_FREQ_STANDARD  100000
#define PIO_I2C_FREQ_FAST      400000
#define PIO_I2C_ACK   0
#define PIO_I2C_NACK  1

class PIO_I2C {
public:
    PIO_I2C(uint8_t sda_pin, uint8_t scl_pin,
            uint32_t freq = PIO_I2C_FREQ_STANDARD);

    bool begin();
    void end();
    bool isInitialized() const { return _initialized; }

    // ─── FIFO Interface (pattern from DHTBus) ─────────────────────────
    void     putCommand(uint16_t cmd);
    bool     hasData();
    uint32_t readFIFO();
    void     clearFIFO();

    // ─── Blocking I2C API (with timeout) ─────────────────────────────
    bool write(uint8_t addr, const uint8_t *data, size_t len, bool stop = true);
    bool read(uint8_t addr, uint8_t *data, size_t len, bool stop = true);
    bool writeThenRead(uint8_t addr,
                       const uint8_t *writeData, size_t writeLen,
                       uint8_t *readData, size_t readLen);
    void scan();

private:
    uint8_t  _sda, _scl;
    uint32_t _freq;
    PIO      _pio;
    uint     _sm, _offset;
    bool     _initialized;

    bool _waitRX(uint32_t timeout_us);
    bool _sendAddress(uint8_t addr, bool read);
    void _sendStop();
    void _waitIdle();
};

#endif
