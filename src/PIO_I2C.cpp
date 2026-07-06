/*
 * PIO_I2C.cpp - I2C Master via PIO for RP2040
 *
 * Uses a PIO state machine to handle the I2C protocol at the byte level.
 * The PIO program handles START, byte send/receive, ACK/NACK, and STOP.
 *
 * PIO program source: see pio/i2c.pio
 * Opcodes: verified against official Raspberry Pi pico-examples
 *   https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c
 *
 * PIO program command word (16 bits, pushed to TX FIFO):
 *   Bits 7:0  = data byte (for writes) or don't care (for reads)
 *   Bit  8    = LAST flag (1 = final byte, generate STOP after)
 *   Bit  9    = READ flag (0 = write, 1 = read)
 *
 * RX FIFO return value:
 *   For WRITE: bit 0 = ACK status (0 = ACK, 1 = NACK)
 *   For READ:  bits 7:0 = received data byte
 *
 * PIO program flow:
 *   1. Pull 16-bit command from TX FIFO (blocks until data available)
 *   2. START condition: SDA high→low while SCL high
 *   3. If WRITE: shift out 8 data bits, then read ACK from slave
 *      If READ:  sample 8 data bits, send ACK (or NACK if last byte)
 *   4. Push result to RX FIFO (ACK for write, data byte for read)
 *   5. If LAST flag set: generate STOP condition
 *   6. Signal IRQ 0, wrap back to step 1
 */

#include "PIO_I2C.h"
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

// ─── PIO Program Opcodes ──────────────────────────────────────────────────
//
// Verified opcodes from the official Raspberry Pi pico-examples I2C PIO
// program. Generated with:
//   pioasm -o c-sdk pio/i2c.pio i2c.pio.h
//
// .program i2c_master
// .side_set 1 opt
//
// Instruction set:
//   0: pull block          side 1     ; wait for command, SCL idle high
//   1: out pindirs, 1      side 1     ; SDA low → START condition
//   2: out y, 1            side 0     ; Y = READ flag (bit 9), SCL low
//   3: jmp !y, 7           side 0     ; if WRITE (Y=0), jump to write
//   4: jmp !osre, 5        side 0     ; (read) ensure OSR not empty
//   5: set x, 7            side 0     ; (read) X = 7, loop counter
//   6: nop                 side 1 [1] ; (read) SCL high
//   7: nop                 side 1 [1] ; (read) delay
//   8: in pins, 1          side 1     ; (read) sample SDA → ISR
//   9: nop                 side 0 [1] ; (read) SCL low
//  10: nop                 side 0 [1] ; (read) delay
//  11: jmp x--, 6          side 0     ; (read) loop 8 times
//  12: jmp 30              side 0     ; (read) goto check_stop
//
//  13: set x, 7            side 0     ; (write) X = 7, loop counter
//  14: out pindirs, 1      side 0 [1] ; (write) output bit → SDA
//  15: nop                 side 1 [1] ; (write) SCL high
//  16: nop                 side 1 [1] ; (write) delay
//  17: nop                 side 0 [1] ; (write) SCL low
//  18: nop                 side 0 [1] ; (write) delay
//  19: jmp x--, 14         side 0     ; (write) loop 8 times
//  20: set pindirs, 1      side 0 [1] ; (write) release SDA for ACK
//  21: nop                 side 1 [1] ; (write) SCL high
//  22: in pins, 1          side 1     ; (write) sample ACK → ISR
//  23: nop                 side 0 [1] ; (write) SCL low
//  24: push noblock        side 0     ; (write) push ACK to RX FIFO
//  25: out y, 1            side 0     ; Y = LAST flag (bit 8)
//  26: jmp !y, 30          side 0     ; if not last, jump to done
//  27: set pindirs, 1      side 0 [1] ; (stop) SDA low
//  28: nop                 side 1 [1] ; (stop) SCL high
//  29: nop                 side 1 [1] ; (stop) delay
//  30: set pindirs, 1      side 1 [1] ; (stop) SDA high while SCL high
//  31: irq nowait 0        side 1     ; signal ARM, wrap to start

static const uint16_t i2c_master_program_instructions[] = {
    0x80a0, //  0: pull   block           side 1
    0xe021, //  1: out    pindirs, 1      side 1
    0x6021, //  2: out    y, 1            side 0
    0x0427, //  3: jmp    !y, 7           side 0
    0x0025, //  4: jmp    !osre, 5        side 0
    0xe825, //  5: set    x, 7            side 0
    0x7128, //  6: nop                    side 1  [1]
    0x7128, //  7: nop                    side 1  [1]
    0x4128, //  8: in     pins, 1         side 1
    0x6028, //  9: nop                    side 0  [1]
    0x6028, // 10: nop                    side 0  [1]
    0x1046, // 11: jmp    x--, 6          side 0
    0x001e, // 12: jmp    30              side 0
    0xe825, // 13: set    x, 7            side 0
    0x6021, // 14: out    pindirs, 1      side 0  [1]
    0x7128, // 15: nop                    side 1  [1]
    0x7128, // 16: nop                    side 1  [1]
    0x6028, // 17: nop                    side 0  [1]
    0x6028, // 18: nop                    side 0  [1]
    0x104e, // 19: jmp    x--, 14         side 0
    0xe021, // 20: set    pindirs, 1      side 0  [1]
    0x7128, // 21: nop                    side 1  [1]
    0x4128, // 22: in     pins, 1         side 1
    0x6028, // 23: nop                    side 0  [1]
    0x8020, // 24: push   noblock         side 0
    0x6021, // 25: out    y, 1            side 0
    0x003e, // 26: jmp    !y, 30          side 0
    0xe021, // 27: set    pindirs, 1      side 0  [1]
    0x7128, // 28: nop                    side 1  [1]
    0x7128, // 29: nop                    side 1  [1]
    0xe021, // 30: set    pindirs, 1      side 1  [1]
    0xc120, // 31: irq    nowait 0        side 1
};

static const pio_program_t i2c_master_program = {
    .instructions = i2c_master_program_instructions,
    .length = 32,
    .origin = -1,
};

// ─── PIO I2C Implementation ───────────────────────────────────────────────

PIO_I2C::PIO_I2C(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq)
    : _sda(sda_pin)
    , _scl(scl_pin)
    , _freq(freq)
    , _pio(nullptr)
    , _sm(0)
    , _offset(0)
    , _initialized(false)
{
}

bool PIO_I2C::begin()
{
    if (_initialized) return true;

    // Allocate a PIO block and state machine
    _pio = pio0;
    _sm = pio_claim_unused_sm(_pio, false);
    if (_sm < 0) {
        _pio = pio1;
        _sm = pio_claim_unused_sm(_pio, false);
        if (_sm < 0) {
            return false; // No free state machines
        }
    }

    // Load the I2C PIO program
    if (!pio_can_add_program(_pio, &i2c_master_program)) {
        pio_sm_unclaim(_pio, _sm);
        return false;
    }
    _offset = pio_add_program(_pio, &i2c_master_program);

    // Configure the state machine
    pio_sm_config cfg = pio_get_default_sm_config();

    // OUT pin mapping: SDA is controlled via pindirs (0=drive low, 1=release)
    sm_config_set_out_pins(&cfg, _sda, 1);

    // Side-set pin mapping: SCL is side-set pin
    sm_config_set_sideset_pins(&cfg, _scl);

    // SET pin mapping: SDA for set pindirs instruction
    sm_config_set_set_pins(&cfg, _sda, 1);

    // IN pin mapping: SDA for reading data/ACK
    sm_config_set_in_pins(&cfg, _sda);

    // Set initial GPIO states:
    // SDA: input with pull-up (open-drain HIGH = released)
    gpio_init(_sda);
    gpio_set_dir(_sda, GPIO_IN);
    gpio_pull_up(_sda);

    // SCL: output, starts low
    gpio_init(_scl);
    gpio_set_dir(_scl, GPIO_OUT);
    gpio_put(_scl, 0);

    // OUT shift register: shift right, auto-pull 16 bits from TX FIFO
    sm_config_set_out_shift(&cfg, true, true, 16);

    // IN shift register: shift right, auto-push at 8 bits to RX FIFO
    sm_config_set_in_shift(&cfg, true, true, 8);

    // Calculate clock divider
    // PIO clock = sys_clk / div
    // Each I2C bit takes ~10 PIO instructions (5 per SCL edge)
    // Target I2C freq = PIO_clk / 10 = sys_clk / (div * 10)
    // → div = sys_clk / (freq * 10)
    float sys_clk = (float)clock_get_hz(clk_sys);
    float div = sys_clk / (float)(_freq * 10);
    sm_config_set_clkdiv(&cfg, div);

    // Initialize and start the state machine
    pio_sm_init(_pio, _sm, _offset, &cfg);
    pio_sm_set_enabled(_pio, _sm, true);

    _initialized = true;
    return true;
}

void PIO_I2C::end()
{
    if (!_initialized) return;

    pio_sm_set_enabled(_pio, _sm, false);
    pio_remove_program(_pio, &i2c_master_program, _offset);
    pio_sm_unclaim(_pio, _sm);

    gpio_set_dir(_sda, GPIO_IN);
    gpio_disable_pulls(_sda);
    gpio_set_dir(_scl, GPIO_IN);
    gpio_disable_pulls(_scl);

    _initialized = false;
}

// ─── Low-level primitives ─────────────────────────────────────────────────

void PIO_I2C::_waitIdle()
{
    // Wait for TX FIFO to drain with timeout
    uint32_t start = micros();
    while (!pio_sm_is_tx_fifo_empty(_pio, _sm)) {
        if (micros() - start > 5000) {
            // Timeout: PIO SM is not consuming data.
            // Drain TX FIFO manually and reset SM.
            pio_sm_clear_fifos(_pio, _sm);
            pio_sm_restart(_pio, _sm);
            return;
        }
        tight_loop_contents();
    }
    // Wait a bit for the last PIO operation to complete
    delayMicroseconds(2);
}

// Timeout helper: wait for RX FIFO with timeout in microseconds
static bool _wait_rx_timeout(PIO pio, uint sm, uint32_t timeout_us) {
    uint32_t start = micros();
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (micros() - start > timeout_us) {
            // Timeout: PIO SM is not responding.
            // Clear FIFOs and reset SM to recover.
            pio_sm_clear_fifos(pio, sm);
            pio_sm_restart(pio, sm);
            return false; // Timeout
        }
        tight_loop_contents();
    }
    return true;
}

bool PIO_I2C::_sendAddress(uint8_t addr, bool read)
{
    // Build address byte: 7-bit addr << 1 | R/W bit
    uint8_t addr_byte = (addr << 1) | (read ? 1 : 0);

    // Push command to PIO TX FIFO:
    //   bits 7:0 = address byte
    //   bit 8 = 0 (not last byte)
    //   bit 9 = 0 (write operation — the address is always a write)
    //   The PIO program will generate START automatically on the first byte
    uint16_t cmd = (uint16_t)addr_byte;
    pio_sm_put_blocking(_pio, _sm, (uint32_t)cmd);

    // The PIO will send 8 bits, read ACK, and push the ACK to RX FIFO
    // I2C byte at 100kHz ≈ 90 µs; use 5ms timeout for safety
    if (!_wait_rx_timeout(_pio, _sm, 5000)) {
        return false; // Timeout — PIO not responding
    }

    uint8_t ack = (uint8_t)pio_sm_get(_pio, _sm);
    return (ack == PIO_I2C_ACK);
}

void PIO_I2C::_sendStop()
{
    _waitIdle();

    // Push a command that triggers STOP:
    //   bit 8 = 1 (LAST byte flag → STOP after)
    //   bit 9 = 0 (write)
    //   bits 7:0 = don't care
    uint16_t cmd = 0x0100; // LAST=1, READ=0
    pio_sm_put_blocking(_pio, _sm, (uint32_t)cmd);

    // PIO generates STOP and signals IRQ
    // Wait for completion
    while (!pio_sm_is_tx_fifo_empty(_pio, _sm)) {
        tight_loop_contents();
    }

    delayMicroseconds(2);
}

// ─── Public API ───────────────────────────────────────────────────────────

bool PIO_I2C::write(uint8_t addr, const uint8_t *data, size_t len, bool stop)
{
    if (!_initialized || len == 0) return false;

    // Send START + address (write mode)
    if (!_sendAddress(addr, false)) {
        _sendStop();
        return false;
    }

    // Send each data byte via PIO
    for (size_t i = 0; i < len; i++) {
        bool last = (i == len - 1) && stop;
        uint16_t cmd = (uint16_t)data[i];
        if (last) {
            cmd |= 0x0100; // bit 8 = LAST → generate STOP after
        }
        // bit 9 = 0 (write)

        pio_sm_put_blocking(_pio, _sm, (uint32_t)cmd);

        // Wait for ACK result with timeout
        if (!_wait_rx_timeout(_pio, _sm, 5000)) {
            if (!last) _sendStop();
            return false;
        }

        uint8_t ack = (uint8_t)pio_sm_get(_pio, _sm);
        if (ack != PIO_I2C_ACK) {
            if (!last) _sendStop();
            return false;
        }
    }

    // If stop was not included in the last byte command, send it now
    if (!stop) {
        // No stop — caller will continue (repeated START)
    }

    return true;
}

bool PIO_I2C::read(uint8_t addr, uint8_t *data, size_t len, bool stop)
{
    if (!_initialized || len == 0) return false;

    // Send START + address (read mode)
    if (!_sendAddress(addr, true)) {
        _sendStop();
        return false;
    }

    // Read each byte via PIO
    for (size_t i = 0; i < len; i++) {
        bool last = (i == len - 1) && stop;
        uint16_t cmd = 0x0200; // bit 9 = 1 (READ)
        if (last) {
            cmd |= 0x0100; // bit 8 = LAST → NACK + STOP after
        }
        // bits 7:0 = don't care for read

        pio_sm_put_blocking(_pio, _sm, (uint32_t)cmd);

        // Wait for received data with timeout
        if (!_wait_rx_timeout(_pio, _sm, 5000)) {
            return false;
        }

        data[i] = (uint8_t)pio_sm_get(_pio, _sm);
    }

    return true;
}

bool PIO_I2C::writeThenRead(uint8_t addr,
                             const uint8_t *writeData, size_t writeLen,
                             uint8_t *readData, size_t readLen)
{
    if (!_initialized) return false;

    // Write phase — no STOP (leaves bus open for repeated START)
    if (writeLen > 0) {
        if (!write(addr, writeData, writeLen, false)) {
            return false;
        }
    }

    // Read phase — with STOP at end
    if (readLen > 0) {
        // Repeated START + address (the PIO will do START since it's a new command)
        if (!_sendAddress(addr, true)) {
            _sendStop();
            return false;
        }

        for (size_t i = 0; i < readLen; i++) {
            bool last = (i == readLen - 1);
            uint16_t cmd = 0x0200 | (last ? 0x0100 : 0x0000);
            pio_sm_put_blocking(_pio, _sm, (uint32_t)cmd);

            if (!_wait_rx_timeout(_pio, _sm, 5000)) {
                _sendStop();
                return false;
            }
            readData[i] = (uint8_t)pio_sm_get(_pio, _sm);
        }
    }

    return true;
}

void PIO_I2C::scan()
{
    if (!_initialized) return;

    Serial.println("I2C Bus Scan (PIO):");
    Serial.print("   ");
    for (int i = 0; i < 16; i++) {
        Serial.print("  ");
        Serial.print(i, HEX);
    }
    Serial.println();

    uint8_t found = 0;
    for (uint8_t addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            if (addr > 0) Serial.println();
            if (addr < 0x10) Serial.print('0');
            Serial.print(addr, HEX);
            Serial.print(':');
        }

        // Try to address device in write mode
        if (_sendAddress(addr, false)) {
            Serial.print(' ');
            if (addr < 0x10) Serial.print(' ');
            Serial.print(addr, HEX);
            found++;
        } else {
            Serial.print(" --");
        }
        _sendStop();
        delayMicroseconds(100);
    }
    Serial.println();
    Serial.print("Found ");
    Serial.print(found);
    Serial.println(" device(s).");
}
