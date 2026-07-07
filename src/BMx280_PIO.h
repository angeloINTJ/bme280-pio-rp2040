/*
 * BMx280_PIO.h - BMP280/BME280 Sensor Driver for RP2040
 * Supports GPIO I2C, hardware Wire, and PIO+DMA auto-scan.
 */
#ifndef BMx280_PIO_H
#define BMx280_PIO_H

#include <Arduino.h>
#include <Wire.h>
#include "PIO_I2C.h"

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

class BMx280_PIO {
public:
    BMx280_PIO(TwoWire &wire, uint8_t addr = BME280_ADDR_PRIMARY);
    BMx280_PIO(uint8_t sda, uint8_t scl, uint8_t addr = BME280_ADDR_PRIMARY, uint32_t freq = 100000);
    ~BMx280_PIO();

    bool begin();
    bool beginPIO(PIO pio = pio0);

    void setMode(uint8_t mode);
    bool takeForcedMeasurement();

    float readTemperature();
    float readPressure();
    float readHumidity();
    void  readAll(float *t, float *p, float *h);

    // PIO+DMA auto-scan (zero-CPU-overhead)
    bool beginAutoScan(uint32_t period_ms = 1000);
    void stopAutoScan();
    void readAllAsync(float *t, float *p, float *h);

    uint8_t getChipID();
    bool isBME280() const { return _is_bme; }
    bool isInitialized() const { return _init; }

private:
    enum Transport { TRANSPORT_WIRE, TRANSPORT_PIO };
    Transport _transport;
    TwoWire *_wire;
    PIO_I2C *_pio_i2c;
    uint8_t  _addr;
    bool     _init, _is_bme;
    uint8_t  _osrs_t, _osrs_p, _osrs_h, _filter, _standby, _mode;
    uint16_t _T1; int16_t _T2, _T3;
    uint16_t _P1; int16_t _P2, _P3, _P4, _P5, _P6, _P7, _P8, _P9;
    uint8_t  _H1; int16_t _H2; uint8_t _H3; int16_t _H4, _H5; int8_t _H6;
    int32_t  _t_fine;
    uint32_t _raw_async[11]; // DMA ring buffer (3 ACKs + 8 data)
    uint8_t  _raw_bytes[8];

    bool _i2c_write(uint8_t reg, const uint8_t *data, size_t len);
    bool _i2c_read(uint8_t reg, uint8_t *data, size_t len);
    bool _loadCalibration();
    void _applyConfig();
    void _readRaw(int32_t *t, int32_t *p, int32_t *h);
    uint8_t _measTime();
};

#endif
