/*
 * BMx280_PIO.cpp - BMP280/BME280 sensor driver implementation
 *
 * Compensation: double-precision floating point (Bosch datasheet Section 4.2.3)
 * Verified against hardware BMP280 on RP2040 at 1014 hPa, 17°C.
 *
 * Transport: hardware I2C (Wire) or PIO-based I2C.
 */

#include "BMx280_PIO.h"

// ─── Constructors ──────────────────────────────────────────────────────────

BMx280_PIO::BMx280_PIO(TwoWire &wire, uint8_t addr)
    : _transport(TRANSPORT_WIRE), _wire(&wire), _pio_i2c(nullptr), _addr(addr)
    , _init(false), _is_bme(false)
    , _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X)
    , _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP)
{
}

BMx280_PIO::BMx280_PIO(uint8_t sda, uint8_t scl, uint8_t addr, uint32_t freq)
    : _transport(TRANSPORT_PIO), _wire(nullptr), _pio_i2c(new PIO_I2C(sda, scl, freq))
    , _addr(addr), _init(false), _is_bme(false)
    , _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X)
    , _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP)
{
}

BMx280_PIO::~BMx280_PIO() {
    if (_pio_i2c) { _pio_i2c->end(); delete _pio_i2c; }
}

// ─── I2C Transport Abstraction ─────────────────────────────────────────────

bool BMx280_PIO::_i2c_write(uint8_t reg, const uint8_t *data, size_t len) {
    if (_transport == TRANSPORT_WIRE) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        for (size_t i = 0; i < len; i++) _wire->write(data[i]);
        return _wire->endTransmission() == 0;
    } else {
        uint8_t buf[len + 1];
        buf[0] = reg;
        for (size_t i = 0; i < len; i++) buf[i + 1] = data[i];
        return _pio_i2c->write(_addr, buf, len + 1);
    }
}

bool BMx280_PIO::_i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    if (_transport == TRANSPORT_WIRE) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        _wire->requestFrom(_addr, len);
        size_t n = _wire->available();
        for (size_t i = 0; i < n && i < len; i++) data[i] = _wire->read();
        return n == len;
    } else {
        return _pio_i2c->writeThenRead(_addr, &reg, 1, data, len);
    }
}

// ─── Initialization ────────────────────────────────────────────────────────

bool BMx280_PIO::begin() {
    if (_init) return true;

    // Init transport
    if (_transport == TRANSPORT_PIO) {
        if (!_pio_i2c->begin()) return false;
    }

    // Reset sensor
    uint8_t reset_val = BME280_RESET_VALUE;
    _i2c_write(BME280_REG_RESET, &reset_val, 1);
    delay(15);

    // Read chip ID
    uint8_t cid = 0;
    if (!_i2c_read(BME280_REG_CHIP_ID, &cid, 1)) return false;

    _is_bme = (cid == BME280_CHIP_ID_VALUE);
    if (cid != BMP280_CHIP_ID && cid != BME280_CHIP_ID_VALUE) return false;

    // Load calibration
    if (!_loadCalibration()) return false;

    // Apply config
    _applyConfig();

    _init = true;
    return true;
}

bool BMx280_PIO::_loadCalibration() {
    uint8_t b[26];
    if (!_i2c_read(BME280_REG_DIG_T1, b, 26)) return false;

    _T1 = b[0] | (b[1] << 8);
    _T2 = b[2] | (b[3] << 8);
    _T3 = b[4] | (b[5] << 8);
    _P1 = b[6] | (b[7] << 8);
    _P2 = b[8] | (b[9] << 8);
    _P3 = b[10] | (b[11] << 8);
    _P4 = b[12] | (b[13] << 8);
    _P5 = b[14] | (b[15] << 8);
    _P6 = b[16] | (b[17] << 8);
    _P7 = b[18] | (b[19] << 8);
    _P8 = b[20] | (b[21] << 8);
    _P9 = b[22] | (b[23] << 8);
    _H1 = b[25];

    if (_is_bme) {
        uint8_t h[8];
        if (!_i2c_read(BME280_REG_DIG_H2, h, 7)) return false;
        _H2 = h[0] | (h[1] << 8);
        _H3 = h[2];
        _H4 = (h[3] << 4) | (h[4] & 0x0F);
        _H5 = (h[4] >> 4) | (h[5] << 4);
        _H6 = h[6];
        if (_H4 > 2047) _H4 -= 4096;
        if (_H5 > 2047) _H5 -= 4096;
    }

    return true;
}

void BMx280_PIO::_applyConfig() {
    uint8_t data;

    // Humidity control (BME280 only)
    if (_is_bme) {
        data = _osrs_h & 0x07;
        _i2c_write(BME280_REG_CTRL_HUM, &data, 1);
    }

    // Measurement control
    data = ((_osrs_t & 0x07) << 5) | ((_osrs_p & 0x07) << 2) | (_mode & 0x03);
    _i2c_write(BME280_REG_CTRL_MEAS, &data, 1);

    // Configuration
    data = ((_standby & 0x07) << 5) | ((_filter & 0x07) << 2);
    _i2c_write(BME280_REG_CONFIG, &data, 1);
}

// ─── Configuration ─────────────────────────────────────────────────────────

void BMx280_PIO::setTemperatureOversampling(uint8_t os) {
    _osrs_t = os & 0x07;
    if (_init) { uint8_t prev = _mode; setMode(BME280_MODE_SLEEP); _applyConfig(); setMode(prev); }
}
void BMx280_PIO::setPressureOversampling(uint8_t os) {
    _osrs_p = os & 0x07;
    if (_init) { uint8_t prev = _mode; setMode(BME280_MODE_SLEEP); _applyConfig(); setMode(prev); }
}
void BMx280_PIO::setHumidityOversampling(uint8_t os) {
    _osrs_h = os & 0x07;
    if (_init) { uint8_t prev = _mode; setMode(BME280_MODE_SLEEP); _applyConfig(); setMode(prev); }
}
void BMx280_PIO::setFilter(uint8_t filter) {
    _filter = filter & 0x07;
    if (_init) { uint8_t prev = _mode; setMode(BME280_MODE_SLEEP); _applyConfig(); setMode(prev); }
}
void BMx280_PIO::setStandbyTime(uint8_t standby) {
    _standby = standby & 0x07;
    if (_init) { uint8_t prev = _mode; setMode(BME280_MODE_SLEEP); _applyConfig(); setMode(prev); }
}
void BMx280_PIO::setMode(uint8_t mode) {
    _mode = mode & 0x03;
    if (!_init) return;
    uint8_t data;
    _i2c_read(BME280_REG_CTRL_MEAS, &data, 1);
    data = (data & 0xFC) | (_mode & 0x03);
    _i2c_write(BME280_REG_CTRL_MEAS, &data, 1);
}

uint8_t BMx280_PIO::_measTime() {
    // Max measurement time in ms
    const uint8_t mult[] = {0, 1, 2, 4, 8, 16};
    float t = 1.25f + 2.3f * mult[_osrs_t & 0x07]
                    + 2.3f * mult[_osrs_p & 0x07] + 0.575f;
    if (_is_bme) t += 2.3f * mult[_osrs_h & 0x07] + 0.575f;
    return (uint8_t)(t + 2.0f);
}

bool BMx280_PIO::takeForcedMeasurement() {
    if (!_init) return false;
    setMode(BME280_MODE_FORCED);
    delay(_measTime());
    uint32_t start = millis();
    uint8_t status;
    do {
        _i2c_read(BME280_REG_STATUS, &status, 1);
        if (millis() - start > 100) return false;
        delay(1);
    } while (status & 0x08);
    return true;
}

// ─── Raw Data ──────────────────────────────────────────────────────────────

void BMx280_PIO::_readRaw(int32_t *t, int32_t *p, int32_t *h) {
    uint8_t d[8];
    _i2c_read(BME280_REG_PRESS_MSB, d, 8);
    if (p) *p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    if (t) *t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    if (h && _is_bme) *h = ((int32_t)d[6] << 8) | d[7];
    else if (h) *h = 0;
}

// ─── Compensation (double-precision, verified correct) ────────────────────

float BMx280_PIO::readTemperature() {
    if (!_init) return NAN;
    int32_t adc_T;
    _readRaw(&adc_T, nullptr, nullptr);

    double v1 = ((double)adc_T / 16384.0 - (double)_T1 / 1024.0) * (double)_T2;
    double v2 = ((double)adc_T / 131072.0 - (double)_T1 / 8192.0);
    v2 = v2 * v2 * (double)_T3;
    _t_fine = (int32_t)(v1 + v2);
    return (float)((v1 + v2) / 5120.0);
}

float BMx280_PIO::readPressure() {
    if (!_init) return NAN;
    int32_t adc_T, adc_P;
    _readRaw(&adc_T, &adc_P, nullptr);

    // Temperature first (sets _t_fine)
    readTemperature();

    double v1 = (double)_t_fine / 2.0 - 64000.0;
    double v2 = v1 * v1 * (double)_P6 / 32768.0;
    v2 += v1 * (double)_P5 * 2.0;
    v2 = v2 / 4.0 + (double)_P4 * 65536.0;
    v1 = ((double)_P3 * v1 * v1 / 524288.0 + (double)_P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * (double)_P1;
    if (v1 == 0.0) return 0.0f;

    double p = 1048576.0 - (double)adc_P;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = (double)_P9 * p * p / 2147483648.0;
    v2 = p * (double)_P8 / 32768.0;
    p += (v1 + v2 + (double)_P7) / 16.0;

    return (float)(p / 100.0);
}

float BMx280_PIO::readHumidity() {
    if (!_init || !_is_bme) return 0.0f;
    int32_t adc_T, adc_H;
    _readRaw(&adc_T, nullptr, &adc_H);

    readTemperature(); // sets _t_fine

    double v = (double)_t_fine - 76800.0;
    double h = ((double)adc_H - ((double)_H4 * 64.0 + (double)_H5 / 16384.0 * v)) *
               ((double)_H2 / 65536.0 *
                (1.0 + (double)_H6 / 67108864.0 * v *
                 (1.0 + (double)_H3 / 67108864.0 * v)));
    h = h * (1.0 - (double)_H1 * h / 524288.0);
    if (h > 100.0) h = 100.0;
    if (h < 0.0)  h = 0.0;
    return (float)h;
}

void BMx280_PIO::readAll(float *t, float *p, float *h) {
    if (!_init) {
        if (t) *t = NAN;
        if (p) *p = NAN;
        if (h) *h = NAN;
        return;
    }
    int32_t aT, aP, aH;
    _readRaw(&aT, &aP, &aH);

    // Temperature
    double v1 = ((double)aT / 16384.0 - (double)_T1 / 1024.0) * (double)_T2;
    double v2 = ((double)aT / 131072.0 - (double)_T1 / 8192.0);
    v2 = v2 * v2 * (double)_T3;
    _t_fine = (int32_t)(v1 + v2);
    float temp = (float)((v1 + v2) / 5120.0);
    if (t) *t = temp;

    // Pressure
    if (p) {
        double pv1 = (double)_t_fine / 2.0 - 64000.0;
        double pv2 = pv1 * pv1 * (double)_P6 / 32768.0;
        pv2 += pv1 * (double)_P5 * 2.0;
        pv2 = pv2 / 4.0 + (double)_P4 * 65536.0;
        pv1 = ((double)_P3 * pv1 * pv1 / 524288.0 + (double)_P2 * pv1) / 524288.0;
        pv1 = (1.0 + pv1 / 32768.0) * (double)_P1;
        if (pv1 == 0.0) {
            *p = 0.0f;
        } else {
            double pp = 1048576.0 - (double)aP;
            pp = (pp - pv2 / 4096.0) * 6250.0 / pv1;
            pv1 = (double)_P9 * pp * pp / 2147483648.0;
            pv2 = pp * (double)_P8 / 32768.0;
            pp += (pv1 + pv2 + (double)_P7) / 16.0;
            *p = (float)(pp / 100.0);
        }
    }

    // Humidity
    if (h) {
        if (!_is_bme) {
            *h = 0.0f;
        } else {
            double hv = (double)_t_fine - 76800.0;
            double hh = ((double)aH - ((double)_H4 * 64.0 + (double)_H5 / 16384.0 * hv)) *
                        ((double)_H2 / 65536.0 *
                         (1.0 + (double)_H6 / 67108864.0 * hv *
                          (1.0 + (double)_H3 / 67108864.0 * hv)));
            hh = hh * (1.0 - (double)_H1 * hh / 524288.0);
            if (hh > 100.0) hh = 100.0;
            if (hh < 0.0)  hh = 0.0;
            *h = (float)hh;
        }
    }
}

// ─── Register Access ───────────────────────────────────────────────────────

uint8_t BMx280_PIO::readRegister(uint8_t reg) {
    uint8_t v = 0;
    _i2c_read(reg, &v, 1);
    return v;
}
void BMx280_PIO::writeRegister(uint8_t reg, uint8_t value) {
    _i2c_write(reg, &value, 1);
}
void BMx280_PIO::readRegisters(uint8_t reg, uint8_t *data, size_t len) {
    _i2c_read(reg, data, len);
}
uint8_t BMx280_PIO::getChipID() {
    return readRegister(BME280_REG_CHIP_ID);
}
void BMx280_PIO::reset() {
    uint8_t v = BME280_RESET_VALUE;
    _i2c_write(BME280_REG_RESET, &v, 1);
}
