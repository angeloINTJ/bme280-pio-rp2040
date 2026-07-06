/*
 * BMx280_PIO.h - BMP280/BME280 sensor driver for RP2040
 *
 * Supports both PIO-based I2C and hardware I2C (Wire) as transport.
 * Auto-detects BMP280 vs BME280 by chip ID.
 * Double-precision compensation verified against Bosch datasheet.
 */

#ifndef BMx280_PIO_H
#define BMx280_PIO_H

#include <Arduino.h>
#include <Wire.h>
#include "PIO_I2C.h"

// ─── I2C Addresses ────────────────────────────────────────────────────────
#define BME280_ADDR_PRIMARY     0x76
#define BME280_ADDR_SECONDARY   0x77

// ─── Register Map ─────────────────────────────────────────────────────────
#define BME280_REG_DIG_T1       0x88
#define BME280_REG_DIG_T2       0x8A
#define BME280_REG_DIG_T3       0x8C
#define BME280_REG_DIG_P1       0x8E
#define BME280_REG_DIG_P2       0x90
#define BME280_REG_DIG_P3       0x92
#define BME280_REG_DIG_P4       0x94
#define BME280_REG_DIG_P5       0x96
#define BME280_REG_DIG_P6       0x98
#define BME280_REG_DIG_P7       0x9A
#define BME280_REG_DIG_P8       0x9C
#define BME280_REG_DIG_P9       0x9E
#define BME280_REG_DIG_H1       0xA1
#define BME280_REG_DIG_H2       0xE1
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_TEMP_MSB     0xFA
#define BME280_REG_HUM_MSB      0xFD

// ─── Chip IDs ─────────────────────────────────────────────────────────────
#define BMP280_CHIP_ID          0x58
#define BME280_CHIP_ID_VALUE    0x60

// ─── Reset ────────────────────────────────────────────────────────────────
#define BME280_RESET_VALUE      0xB6

// ─── Sensor Modes ─────────────────────────────────────────────────────────
#define BME280_MODE_SLEEP       0x00
#define BME280_MODE_FORCED      0x01
#define BME280_MODE_NORMAL      0x03

// ─── Oversampling ─────────────────────────────────────────────────────────
#define BME280_OS_SKIP          0x00
#define BME280_OS_1X            0x01
#define BME280_OS_2X            0x02
#define BME280_OS_4X            0x03
#define BME280_OS_8X            0x04
#define BME280_OS_16X           0x05

// ─── Filter ───────────────────────────────────────────────────────────────
#define BME280_FILTER_OFF       0x00
#define BME280_FILTER_2         0x01
#define BME280_FILTER_4         0x02
#define BME280_FILTER_8         0x03
#define BME280_FILTER_16        0x04

// ─── Standby ──────────────────────────────────────────────────────────────
#define BME280_STANDBY_0_5MS    0x00
#define BME280_STANDBY_250MS    0x03
#define BME280_STANDBY_500MS    0x04
#define BME280_STANDBY_1000MS   0x05

/*
 * BMx280_PIO - BMP280/BME280 sensor driver.
 *
 * Two transport modes:
 *   1. Hardware I2C (default): BMx280_PIO sensor(Wire1, 0x76)
 *   2. PIO I2C: BMx280_PIO sensor(sda, scl, 0x76)
 *
 * Usage:
 *   // Hardware I2C (Wire1 on GP2/GP3):
 *   Wire1.setSDA(2); Wire1.setSCL(3); Wire1.begin();
 *   BMx280_PIO sensor(Wire1, 0x76);
 *   sensor.begin();
 *   float t = sensor.readTemperature();
 *   float p = sensor.readPressure();
 */
class BMx280_PIO {
public:
    // ─── Constructors ─────────────────────────────────────────────────
    /* Hardware I2C transport (TwoWire) */
    BMx280_PIO(TwoWire &wire, uint8_t addr = BME280_ADDR_PRIMARY);

    /* PIO I2C transport (any GPIO pins) */
    BMx280_PIO(uint8_t sda, uint8_t scl,
               uint8_t addr = BME280_ADDR_PRIMARY,
               uint32_t freq = 100000);

    ~BMx280_PIO();

    // ─── Initialization ───────────────────────────────────────────────
    bool begin();

    // ─── Configuration ────────────────────────────────────────────────
    void setTemperatureOversampling(uint8_t os);
    void setPressureOversampling(uint8_t os);
    void setHumidityOversampling(uint8_t os);
    void setFilter(uint8_t filter);
    void setStandbyTime(uint8_t standby);
    void setMode(uint8_t mode);
    bool takeForcedMeasurement();

    // ─── Readings (compensated) ───────────────────────────────────────
    float readTemperature();           // °C
    float readPressure();              // hPa
    float readHumidity();              // % (0 if BMP280)
    void  readAll(float *t, float *p, float *h);

    // ─── Sensor Info ──────────────────────────────────────────────────
    uint8_t getChipID();
    bool    isBME280() const { return _is_bme; }
    bool    isInitialized() const { return _init; }

    // ─── Register Access ──────────────────────────────────────────────
    uint8_t readRegister(uint8_t reg);
    void    writeRegister(uint8_t reg, uint8_t value);
    void    readRegisters(uint8_t reg, uint8_t *data, size_t len);
    void    reset();

private:
    // Transport
    enum Transport { TRANSPORT_WIRE, TRANSPORT_PIO };
    Transport _transport;

    TwoWire *_wire;
    PIO_I2C *_pio_i2c;
    uint8_t  _addr;

    bool     _init;
    bool     _is_bme;

    // Configuration
    uint8_t  _osrs_t, _osrs_p, _osrs_h;
    uint8_t  _filter, _standby, _mode;

    // Calibration
    uint16_t _T1; int16_t _T2, _T3;
    uint16_t _P1; int16_t _P2, _P3, _P4, _P5, _P6, _P7, _P8, _P9;
    uint8_t  _H1; int16_t _H2; uint8_t _H3; int16_t _H4, _H5; int8_t _H6;
    int32_t  _t_fine;

    // Internal methods
    bool _i2c_write(uint8_t reg, const uint8_t *data, size_t len);
    bool _i2c_read(uint8_t reg, uint8_t *data, size_t len);
    bool _loadCalibration();
    void _applyConfig();
    void _readRaw(int32_t *t, int32_t *p, int32_t *h);
    uint8_t _measTime();
};

#endif // BMx280_PIO_H
