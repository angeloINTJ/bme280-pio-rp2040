/**
 * @file BMx280PIO_RP2040.h
 * @brief BMP280/BME280 sensor driver for RP2040 — I2C via WirePIO or hardware Wire.
 *
 * Auto-detects BMP280 vs BME280 by chip ID. Uses TwoWirePIO_RP2040 (WirePIO)
 * for PIO+DMA I2C on any GPIO pin pair, or hardware I2C via TwoWire&.
 *
 * @author angeloINTJ
 * @license MIT
 */
#ifndef BMX280PIO_RP2040_H
#define BMX280PIO_RP2040_H

#include <Arduino.h>
#include <Wire.h>
#include <hardware/pio.h>

class WirePIO;

#define BME280_ADDR_PRIMARY     0x76
#define BME280_ADDR_SECONDARY   0x77
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5

#define BMP280_CHIP_ID          0x58
#define BME280_CHIP_ID_VALUE    0x60
#define BME280_RESET_VALUE      0xB6

#define BME280_MODE_SLEEP       0x00
#define BME280_MODE_FORCED      0x01
#define BME280_MODE_NORMAL      0x03

#define BME280_OS_SKIP          0x00
#define BME280_OS_1X            0x01
#define BME280_OS_2X            0x02
#define BME280_OS_4X            0x03
#define BME280_OS_8X            0x04
#define BME280_OS_16X           0x05

#define BME280_FILTER_OFF       0x00
#define BME280_FILTER_4         0x02
#define BME280_STANDBY_250MS    0x03
#define BME280_STANDBY_500MS    0x04
#define BME280_STANDBY_1000MS   0x05

class BMx280PIO_RP2040 {
public:
    BMx280PIO_RP2040(TwoWire &wire, uint8_t addr = BME280_ADDR_PRIMARY);
    BMx280PIO_RP2040(uint8_t sda, uint8_t scl, uint8_t addr = BME280_ADDR_PRIMARY,
                     uint32_t freq = 400000, PIO pio = pio0);
    ~BMx280PIO_RP2040();

    bool begin();
    bool beginPIO(PIO pio = pio0);

    void setTemperatureOversampling(uint8_t os) { _osrs_t = os & 0x07; if (_init) _applyConfig(); }
    void setPressureOversampling(uint8_t os)    { _osrs_p = os & 0x07; if (_init) _applyConfig(); }
    void setHumidityOversampling(uint8_t os)    { _osrs_h = os & 0x07; if (_init) _applyConfig(); }
    void setFilter(uint8_t filter)              { _filter = filter & 0x07; if (_init) _applyConfig(); }
    void setStandbyTime(uint8_t standby)        { _standby = standby & 0x07; if (_init) _applyConfig(); }
    void setMode(uint8_t mode);
    bool takeForcedMeasurement();

    uint8_t readRegister(uint8_t reg);
    void    writeRegister(uint8_t reg, uint8_t value);
    void    readRegisters(uint8_t reg, uint8_t *data, size_t len);

    float readTemperature();
    float readPressure();
    float readHumidity();
    void  readAll(float *t, float *p, float *h);

    uint8_t getChipID();
    bool isBME280() const { return _is_bme; }
    bool isInitialized() const { return _init; }

private:
    TwoWire  *_wire;
    WirePIO  *_wirepio;
    uint8_t   _addr, _sda, _scl;
    uint32_t  _freq;
    bool      _init, _is_bme;
    uint8_t   _osrs_t, _osrs_p, _osrs_h, _filter, _standby, _mode;

    uint16_t _T1; int16_t _T2, _T3;
    uint16_t _P1; int16_t _P2, _P3, _P4, _P5, _P6, _P7, _P8, _P9;
    uint8_t  _H1; int16_t _H2; uint8_t _H3; int16_t _H4, _H5; int8_t _H6;
    int32_t  _t_fine;

    bool _i2c_write(uint8_t reg, const uint8_t *data, size_t len);
    bool _i2c_read(uint8_t reg, uint8_t *data, size_t len);
    bool _loadCalibration();
    void _applyConfig();
    void _readRaw(int32_t *t, int32_t *p, int32_t *h);
    uint8_t _measTime();
};

#endif
