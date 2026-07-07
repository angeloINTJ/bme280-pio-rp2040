/*
 * PIO_I2C.h - I2C Master for RP2040
 *
 * GPIO bit-bang + PIO/DMA burst transport.
 * Uses original i2c.pio (pico-examples based) with LSB-first shift.
 */

#ifndef PIO_I2C_H
#define PIO_I2C_H

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/pwm.h>

#define PIO_I2C_FREQ_STANDARD  100000
#define PIO_I2C_FREQ_FAST      400000

class PIO_I2C {
public:
    PIO_I2C(uint8_t sda, uint8_t scl, uint32_t freq = PIO_I2C_FREQ_STANDARD);
    ~PIO_I2C();

    bool begin();
    void end();
    bool isInitialized() const { return _initialized; }

    // GPIO bit-bang API
    bool write(uint8_t addr, const uint8_t *data, size_t len, bool stop = true);
    bool read(uint8_t addr, uint8_t *data, size_t len, bool stop = true);
    bool writeThenRead(uint8_t addr,
                       const uint8_t *writeData, size_t writeLen,
                       uint8_t *readData, size_t readLen);
    void scan();

    // PIO + DMA setup
    bool beginPIO(PIO pio = pio0);
    bool isPIOActive() const { return _pio_active; }

    // 3-Channel DMA continuous scan
    bool beginAutoScan(uint8_t addr, uint8_t reg,
                       uint32_t *buf, size_t len, uint32_t period_ms);
    void stopAutoScan();
    bool isAutoScanActive() const { return _auto_scan; }

    // Extract bytes from 32-bit DMA words (LSB-first: byte in bits 7:0)
    static void extractBytes(const uint32_t *src, uint8_t *dst, size_t len);

private:
    uint8_t  _sda, _scl;
    uint32_t _freq;
    PIO      _pio;
    int      _sm, _offset;
    bool     _initialized;
    bool     _pio_active;
    bool     _auto_scan;
    int      _dma_tx_chan, _dma_rx_chan, _dma_ctrl_chan;
    int      _pwm_slice;
    uint     _burst_len;
    uint32_t *_burst_buf;

    static const size_t MAX_CMDS = 3 + 32;
    uint32_t _cmd_buf[MAX_CMDS];
    size_t   _cmd_count;
    uint32_t _ctrl_data[2];

    // Helpers
    void _buildBurstCommands(uint8_t addr, uint8_t reg, size_t len);
    void _setupDMA();
    void _setupPWM(uint32_t period_ms);
    bool _sendAddress(uint8_t addr, bool read);
    void _sendStop();
    void _waitIdle();
};

#endif
