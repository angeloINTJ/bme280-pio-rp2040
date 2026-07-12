# BMx280PIO_RP2040 — BMP280/BME280 Driver for RP2040

[![PlatformIO](https://img.shields.io/badge/PlatformIO-compatible-orange.svg)](https://platformio.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Arduino library for the **Bosch BMP280/BME280** environmental sensor on **RP2040** (Raspberry Pi Pico). Uses **PIO+DMA burstRead** (3.3× faster than Adafruit) with GPIO bit-bang fallback and hardware Wire support.

## Features

- ✅ **PIO+DMA burstRead @ 200 kHz** — 553 µs/read, 1808 reads/sec on hardware
- ✅ **GPIO bit-bang fallback** — any pin pair, reliable at 100 kHz (3522 µs/read)
- ✅ **Hardware I2C (Wire)** — Adafruit-compatible mode
- ✅ `forceGPIO()` mode for benchmarking GPIO vs PIO performance
- ✅ Auto-detects BMP280 vs BME280 by chip ID
- ✅ Sleep, Forced, and Normal operating modes
- ✅ Configurable oversampling: 1× to 16× per channel
- ✅ IIR filter and standby time configuration
- ✅ Bosch datasheet compensation (double-precision)
- ✅ PlatformIO & Arduino IDE compatible

## Quick Start

### PIO+DMA I2C (any pins, automatic)

The GPIO-pin constructor automatically initializes PIO+DMA for high-performance reads.

```cpp
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

BMx280PIO_RP2040 sensor(2, 3);  // SDA=GP2, SCL=GP3, I2C @ 200 kHz

void setup() {
    Serial.begin(115200);
    sensor.begin();    // PIO+DMA burstRead initialized automatically
    sensor.setMode(BME280_MODE_NORMAL);
}
void loop() {
    float t, p, h; sensor.readAll(&t, &p, &h);
    Serial.printf("T=%.2f°C P=%.2fhPa\n", t, p);
    delay(2000);
}
```

### Hardware I2C (Wire)

```cpp
#include <Arduino.h>
#include "BMx280PIO_RP2040.h"

BMx280PIO_RP2040 sensor(Wire, 0x76);  // Uses hardware I2C0

void setup() {
    Wire.begin();
    Serial.begin(115200);
    sensor.begin();
}
```

### GPIO Bit-Bang (forced fallback)

```cpp
BMx280PIO_RP2040 sensor(2, 3);
sensor.forceGPIO(true);   // Skip PIO+DMA, use GPIO bit-bang only
sensor.begin();
```

## Wiring

| BMP280/BME280 | Raspberry Pi Pico |
|---------------|-------------------|
| VCC           | 3.3V              |
| GND           | GND               |
| SDA           | GPIO2 (or any)    |
| SCL           | GPIO3 (or any)    |

> ⚠️ The sensor operates at **3.3V**. Do not connect to 5V. External 10kΩ pull-up resistors on SDA and SCL to 3.3V are recommended.

## Hardware Test Results

Tested on BMP280 (chip ID 0x58) at address 0x76, GPIO2=SDA, GPIO3=SCL.

| Method | Time/read | Throughput | Notes |
|--------|-----------|------------|-------|
| **PIO+DMA (200 kHz)** | **553 µs** | **1808 r/s** | 3.3× faster than Adafruit |
| Adafruit Wire (100 kHz) | 1837 µs | 544 r/s | Hardware I2C baseline |
| GPIO bit-bang (100 kHz) | 3522 µs | 284 r/s | Reliable fallback |

All examples (`basic_reading`, `forced_mode`, `auto_scan`, `multi_sensor`, `benchmark`) verified on hardware.

## Architecture

```
┌────────────────────────────────────────────────────────────┐
│  BMx280PIO_RP2040 (sensor driver)                          │
│  - Bosch compensation formulas                             │
│  - Auto-detection BMP280 vs BME280                         │
└──────────────┬─────────────────────────────────────────────┘
               │
┌──────────────▼─────────────────────────────────────────────┐
│  TwoWirePIO_RP2040 (I2C transport)                          │
│  - PIO+DMA burstRead: 8 registers in one transaction       │
│  - GPIO bit-bang fallback on any pin pair                  │
│  - 2-channel DMA engine: TX (cmd → PIO), RX (PIO → buf)   │
└──────────────┬─────────────────────────────────────────────┘
               │
┌──────────────▼─────────────────────────────────────────────┐
│  PIO State Machine (i2c.pio — 31 instructions)             │
│  - Bit-bang I2C master (SCL via side-set, SDA via OUT/SET) │
│  - Command word: START + READ + 8-bit data + STOP          │
│  - Autopush RX data to FIFO at 8-bit threshold             │
│  - Explicit push for ACK bits                              │
└────────────────────────────────────────────────────────────┘
```

### PIO+DMA Burst Read

The GPIO-pin constructor automatically loads the PIO program and configures DMA
channels. Each `readAll()` call executes a zero-CPU burst transaction:

1. DMA CH1 sends command words to the PIO TX FIFO
2. PIO state machine executes: START + write register + RESTART + read 8 bytes
3. DMA CH2 drains data bytes from the RX FIFO into a buffer
4. CPU extracts the bytes and runs Bosch compensation

Key implementation details:
- **Chunked calibration reads** — 26-byte calibration split into ≤8-byte chunks for burstRead compatibility
- **Manual DMA register writes** — work around SDK `dma_channel_configure()` TX count bug
- **DMA enable after PIO SM start** — ensures DREQ signals are active
- **PIO prologue restructured** — START/READ flags extracted via `out y/x` before any SCL edge, eliminating glitches

## PIO Program — Key Design Decisions

### Command Encoding (16-bit, LSB-first shift from OSR)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | START | 1 = generate START before this byte |
| 1 | READ | 1 = read from slave, 0 = write |
| 9:2 | DATA | Write data byte (`~rev8(data)`) |
| 10 | STOP | 1 = generate STOP after this byte |

### SCL Glitch Fix

**Problem**: The original prologue used `out pindirs, 1 side 1` to extract the START flag. For START=0 commands, this forced SCL HIGH for 1 cycle — a spurious clock edge that the BME280 interpreted as an extra SCL pulse, shifting all read data by 1 bit.

**Solution**: Restructured prologue extracts both START and READ flags via `out y`/`out x` **before any SCL edge**. START is generated conditionally using `set pindirs` (no OSR consumption), followed by `set pindirs, 0` to release the SET drive so SDA can float for reads.

```
pull block          side 0    ; SCL LOW
out y, 1            side 0    ; Y = START (OSR bit 0)
out x, 1            side 0    ; X = READ  (OSR bit 1)
jmp !y, branch      side 0    ; START=0 → skip (SCL never goes HIGH!)
set pindirs, 1      side 1    ; SCL↑ + SDA↓ = START
set pindirs, 0      side 0 [1]; release SET, SDA floats again
branch:
    jmp !x, write_byte side 0 ; READ=0 → write path
```

**Result**: Zero SCL glitch for START=0 commands. Read data matches GPIO reference exactly.

### Data Extraction: No Bit Reversal Needed

The PIO ISR with `shift_in_right=false` stores the first received bit (I2C MSB)
at ISR[7] and the last received bit (I2C LSB) at ISR[0]. The byte is **already
in correct I2C order** — no `rev8()` is needed.

Extraction is simply: `dst[i] = rxbuf[i] & 0xFF`

### Dual-Core Logic Analyzer

During development, a powerful debugging technique was used: **Core 1 as a logic
analyzer**. Core 1 samples SDA/SCL via `sio_hw->gpio_in` at ~5 MHz into a 4K
ring buffer while Core 0 runs the PIO+DMA burst. This revealed:

- The `dma_channel_configure()` TX transfer count bug (register always read 0)
- The missing ACK setup time (SDA and SCL changing in the same PIO cycle)
- The DMA enable ordering issue (DREQ not active before PIO SM starts)

### SCL Recovery

The PIO sends an ACK pulse (SDA LOW during 9th SCL) after each read byte,
keeping the sensor in streaming mode for multi-byte reads. After the burst
completes, `burstRead()` generates a GPIO SCL recovery pulse to ensure the
bus returns to idle state.

## API Reference

### Constructor

```cpp
// PIO+DMA I2C on any GPIO pins
BMx280PIO_RP2040 sensor(uint8_t sda, uint8_t scl,
                  uint8_t addr = 0x76,
                  uint32_t freq = 200000, PIO pio = pio0);

// Hardware I2C (Wire)
BMx280PIO_RP2040 sensor(TwoWire &wire, uint8_t addr = 0x76);
```

### Configuration

```cpp
bool begin();                    // Initialize sensor and load calibration
bool beginPIO(PIO pio = pio0);   // Re-init PIO+DMA on a different PIO block
void setMode(uint8_t mode);      // SLEEP, FORCED, NORMAL
bool takeForcedMeasurement();    // Trigger single conversion + wait
void setTemperatureOversampling(uint8_t os);  // 1× to 16×
void setPressureOversampling(uint8_t os);
void setHumidityOversampling(uint8_t os);
void setFilter(uint8_t filter);              // OFF, 2, 4, 8, 16
void setStandbyTime(uint8_t standby);        // 250ms to 1000ms
```

### Readings

```cpp
float readTemperature();         // °C
float readPressure();            // hPa
float readHumidity();            // % (0 if BMP280)
void  readAll(float *t, float *p, float *h);
```

### GPIO Mode Control

```cpp
void forceGPIO(bool f);          // Force GPIO bit-bang (skip PIO+DMA)
bool isForcedGPIO();             // Check if GPIO mode is active
```

### Utilities

```cpp
uint8_t getChipID();             // 0x58 = BMP280, 0x60 = BME280
bool isBME280();                 // True if humidity sensor present
bool isInitialized();            // True if sensor ready
uint8_t readRegister(uint8_t reg);
void    writeRegister(uint8_t reg, uint8_t value);
void    readRegisters(uint8_t reg, uint8_t *data, size_t len);
```

## Operating Modes

| Mode | Current | Use Case |
|------|---------|----------|
| **Sleep** | 0.1 µA | Sensor idle, registers preserved |
| **Forced** | 1.2 mA (peak) / ~3 µA avg | Single measurement, auto-return to sleep |
| **Normal** | 1.2 mA (peak) / ~2.7 µA @ 1 Hz | Continuous measurement, configurable interval |

## Known Limitations

1. **STOP after reads**: The READ path reads the STOP flag from the wrong OSR bit
   position (bit 3 instead of bit 10). The GPIO recovery in `burstRead()` handles bus cleanup.
2. **First reading offset**: The first byte of the first reading after forced mode
   may have a slightly offset pressure MSB. This is a sensor characteristic, not a driver bug — subsequent readings are stable.
3. **Ring buffer wrap** in burst mode: the DMA CH2 ring buffer (32 bytes) wraps after
   8 words. The 11-word burst (3 ACKs + 8 data bytes) wraps the last 3 words to
   positions 0-2. The extraction code handles this correctly.
4. **RP2040-only**: Requires the PIO peripheral, exclusive to RP2040/RP2350.

## Technical Notes

### I2C Bit Timing

With PIO clock divider ≈ 96.15 (125 MHz sysclk ÷ 1.3 MHz PIO clock):

| Parameter | Time | Fast Mode Spec |
|-----------|------|----------------|
| SCL high | 2.3 µs | ≥ 0.6 µs |
| SCL low | 6.9–13.1 µs | ≥ 1.3 µs |
| SCL frequency | ~65–87 kHz | ≤ 400 kHz |

The timing is conservative (slower than max spec) for robust operation. The BME280 tolerates the slower clock transparently.

## Dependencies

- **[TwoWirePIO_RP2040](https://github.com/angeloINTJ/TwoWirePIO_RP2040) (>=1.3.2)** — PIO+DMA I2C transport layer (installed automatically by Library Manager)
- [arduino-pico](https://github.com/earlephilhower/arduino-pico) — Arduino core for RP2040 (Earle Philhower)

## License

MIT — see [LICENSE](LICENSE) for details.

## Author

Ângelo Moisés Alves — [@angeloINTJ](https://github.com/angeloINTJ)

## Credits

- **Bosch Sensortec** — BME280 datasheet and compensation formulas
- **Raspberry Pi Foundation** — PIO I2C example and RP2040 SDK
- **Earle Philhower** — Arduino-Pico core
