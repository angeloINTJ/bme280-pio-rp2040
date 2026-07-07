/*
 * BMx280_PIO.cpp - BMP280/BME280 Sensor Driver
 * Bosch compensation formulas. GPIO I2C + PIO+DMA auto-scan.
 */
#include "BMx280_PIO.h"

// Bit-reverse an 8-bit byte (I2C MSB-first → PIO ISR LSB-first correction)
static uint8_t rev8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

BMx280_PIO::BMx280_PIO(TwoWire &wire, uint8_t addr)
    : _transport(TRANSPORT_WIRE), _wire(&wire), _pio_i2c(nullptr), _addr(addr),
      _init(false), _is_bme(false),
      _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X),
      _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP) {}

BMx280_PIO::BMx280_PIO(uint8_t sda, uint8_t scl, uint8_t addr, uint32_t freq)
    : _transport(TRANSPORT_PIO), _wire(nullptr), _pio_i2c(new PIO_I2C(sda, scl, freq)),
      _addr(addr), _init(false), _is_bme(false),
      _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X),
      _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP) {}

BMx280_PIO::~BMx280_PIO() { if (_pio_i2c) { _pio_i2c->end(); delete _pio_i2c; } }

bool BMx280_PIO::_i2c_write(uint8_t reg, const uint8_t *data, size_t len) {
    if (_transport == TRANSPORT_WIRE) {
        _wire->beginTransmission(_addr); _wire->write(reg);
        for (size_t i = 0; i < len; i++) _wire->write(data[i]);
        return _wire->endTransmission() == 0;
    } else {
        uint8_t buf[len + 1]; buf[0] = reg;
        for (size_t i = 0; i < len; i++) buf[i + 1] = data[i];
        return _pio_i2c->write(_addr, buf, len + 1);
    }
}
bool BMx280_PIO::_i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    if (_transport == TRANSPORT_WIRE) {
        _wire->beginTransmission(_addr); _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        _wire->requestFrom(_addr, len);
        size_t n = _wire->available();
        for (size_t i = 0; i < n && i < len; i++) data[i] = _wire->read();
        return n == len;
    } else {
        return _pio_i2c->writeThenRead(_addr, &reg, 1, data, len);
    }
}

bool BMx280_PIO::begin() {
    if (_init) return true;
    if (_transport == TRANSPORT_PIO && !_pio_i2c->begin()) return false;
    uint8_t rst = BME280_RESET_VALUE;
    _i2c_write(BME280_REG_RESET, &rst, 1);
    sleep_ms(15);
    uint8_t cid = 0;
    if (!_i2c_read(BME280_REG_CHIP_ID, &cid, 1)) return false;
    _is_bme = (cid == BME280_CHIP_ID_VALUE);
    if (cid != BMP280_CHIP_ID && cid != BME280_CHIP_ID_VALUE) return false;
    if (!_loadCalibration()) return false;
    _applyConfig();
    _init = true; return true;
}

bool BMx280_PIO::_loadCalibration() {
    uint8_t b[26];
    if (!_i2c_read(0x88, b, 26)) return false;
    _T1 = b[0]|(b[1]<<8); _T2 = b[2]|(b[3]<<8); _T3 = b[4]|(b[5]<<8);
    _P1 = b[6]|(b[7]<<8); _P2 = b[8]|(b[9]<<8); _P3 = b[10]|(b[11]<<8);
    _P4 = b[12]|(b[13]<<8); _P5 = b[14]|(b[15]<<8); _P6 = b[16]|(b[17]<<8);
    _P7 = b[18]|(b[19]<<8); _P8 = b[20]|(b[21]<<8); _P9 = b[22]|(b[23]<<8);
    _H1 = b[25];
    if (_is_bme) {
        uint8_t h[8];
        if (!_i2c_read(0xE1, h, 7)) return false;
        _H2 = h[0]|(h[1]<<8); _H3 = h[2];
        _H4 = (h[3]<<4)|(h[4]&0x0F); _H5 = (h[4]>>4)|(h[5]<<4); _H6 = h[6];
        if (_H4 > 2047) _H4 -= 4096; if (_H5 > 2047) _H5 -= 4096;
    }
    return true;
}

void BMx280_PIO::_applyConfig() {
    uint8_t d;
    if (_is_bme) { d = _osrs_h & 0x07; _i2c_write(BME280_REG_CTRL_HUM, &d, 1); }
    d = ((_osrs_t & 0x07) << 5) | ((_osrs_p & 0x07) << 2) | (_mode & 0x03);
    _i2c_write(BME280_REG_CTRL_MEAS, &d, 1);
    d = ((_standby & 0x07) << 5) | ((_filter & 0x07) << 2);
    _i2c_write(BME280_REG_CONFIG, &d, 1);
}

void BMx280_PIO::setMode(uint8_t mode) {
    _mode = mode & 0x03;
    if (!_init) return;
    uint8_t d; _i2c_read(BME280_REG_CTRL_MEAS, &d, 1);
    d = (d & 0xFC) | (_mode & 0x03);
    _i2c_write(BME280_REG_CTRL_MEAS, &d, 1);
}

uint8_t BMx280_PIO::_measTime() {
    const uint8_t m[] = {0, 1, 2, 4, 8, 16};
    uint32_t t = 1250 + 2300 * m[_osrs_t & 0x07] + 2300 * m[_osrs_p & 0x07] + 575;
    if (_is_bme) t += 2300 * m[_osrs_h & 0x07] + 575;
    return (uint8_t)((t + 1500) / 1000);
}

bool BMx280_PIO::takeForcedMeasurement() {
    if (!_init) return false;
    setMode(BME280_MODE_FORCED);
    sleep_ms(_measTime());
    uint32_t start = millis();
    uint8_t st;
    do {
        if (!_i2c_read(BME280_REG_STATUS, &st, 1)) return false;
        if (millis() - start > 100) return false;
        sleep_ms(1);
    } while (st & 0x08);
    return true;
}

void BMx280_PIO::_readRaw(int32_t *t, int32_t *p, int32_t *h) {
    uint8_t d[8];
    _i2c_read(BME280_REG_PRESS_MSB, d, 8);
    if (p) *p = ((int32_t)d[0]<<12)|((int32_t)d[1]<<4)|(d[2]>>4);
    if (t) *t = ((int32_t)d[3]<<12)|((int32_t)d[4]<<4)|(d[5]>>4);
    if (h && _is_bme) *h = ((int32_t)d[6]<<8)|d[7];
    else if (h) *h = 0;
}

float BMx280_PIO::readTemperature() {
    if (!_init) return NAN;
    int32_t adc_T; _readRaw(&adc_T, nullptr, nullptr);
    int32_t v1 = ((((adc_T>>3)-((int32_t)_T1<<1)))*_T2)>>11;
    int32_t v2 = (((((adc_T>>4)-_T1)*((adc_T>>4)-_T1))>>12)*_T3)>>14;
    _t_fine = v1 + v2;
    return (float)((_t_fine*5+128)>>8)/100.0f;
}
float BMx280_PIO::readPressure() {
    if (!_init) return NAN;
    int32_t adc_T, adc_P; _readRaw(&adc_T, &adc_P, nullptr);
    readTemperature();
    double v1 = (double)_t_fine/2.0-64000.0;
    double v2 = v1*v1*(double)_P6/32768.0;
    v2 += v1*(double)_P5*2.0;
    v2 = v2/4.0+(double)_P4*65536.0;
    v1 = ((double)_P3*v1*v1/524288.0+(double)_P2*v1)/524288.0;
    v1 = (1.0+v1/32768.0)*(double)_P1;
    if (v1==0.0) return 0.0f;
    double p = 1048576.0-(double)adc_P;
    p = (p-v2/4096.0)*6250.0/v1;
    v1 = (double)_P9*p*p/2147483648.0;
    v2 = p*(double)_P8/32768.0;
    p += (v1+v2+(double)_P7)/16.0;
    return (float)(p/100.0);
}
float BMx280_PIO::readHumidity() {
    if (!_init||!_is_bme) return 0.0f;
    int32_t adc_T, adc_H; _readRaw(&adc_T, nullptr, &adc_H);
    readTemperature();
    int32_t v1 = _t_fine-76800;
    v1 = (int32_t)((((((int64_t)adc_H<<14)-((int64_t)_H4<<20)-((int64_t)_H5*v1))+16384)>>15)*
                   ((((((((int64_t)v1*_H6)>>10)*((((int64_t)v1*_H3)>>11)+32768))>>10)+2097152)*_H2+8192)>>14));
    v1 = v1-(((((v1>>15)*(v1>>15))>>7)*(int32_t)_H1)>>4);
    v1 = v1<0?0:v1; v1 = v1>419430400?419430400:v1;
    return (float)((uint32_t)(v1>>12))/1024.0f;
}
void BMx280_PIO::readAll(float *t, float *p, float *h) {
    if (!_init) { if(t)*t=NAN; if(p)*p=NAN; if(h)*h=NAN; return; }
    int32_t aT,aP,aH; _readRaw(&aT,&aP,&aH);
    int32_t tv1=((((aT>>3)-((int32_t)_T1<<1)))*_T2)>>11;
    int32_t tv2=(((((aT>>4)-_T1)*((aT>>4)-_T1))>>12)*_T3)>>14;
    _t_fine=tv1+tv2;
    if(t)*t=(float)((_t_fine*5+128)>>8)/100.0f;
    if(p){
        double pv1=(double)_t_fine/2.0-64000.0;
        double pv2=pv1*pv1*(double)_P6/32768.0;
        pv2+=pv1*(double)_P5*2.0;
        pv2=pv2/4.0+(double)_P4*65536.0;
        pv1=((double)_P3*pv1*pv1/524288.0+(double)_P2*pv1)/524288.0;
        pv1=(1.0+pv1/32768.0)*(double)_P1;
        if(pv1==0.0){*p=0.0f;}else{
            double pp=1048576.0-(double)aP;
            pp=(pp-pv2/4096.0)*6250.0/pv1;
            pv1=(double)_P9*pp*pp/2147483648.0;
            pv2=pp*(double)_P8/32768.0;
            pp+=(pv1+pv2+(double)_P7)/16.0;
            *p=(float)(pp/100.0);
        }
    }
    if(h){
        if(!_is_bme){*h=0.0f;}else{
            int32_t hv=_t_fine-76800;
            hv=(int32_t)((((((int64_t)aH<<14)-((int64_t)_H4<<20)-((int64_t)_H5*hv))+16384)>>15)*
                         ((((((((int64_t)hv*_H6)>>10)*((((int64_t)hv*_H3)>>11)+32768))>>10)+2097152)*_H2+8192)>>14));
            hv=hv-(((((hv>>15)*(hv>>15))>>7)*(int32_t)_H1)>>4);
            hv=hv<0?0:hv;hv=hv>419430400?419430400:hv;
            *h=(float)((uint32_t)(hv>>12))/1024.0f;
        }
    }
}

// ─── PIO+DMA Auto-Scan ─────────────────────────────────────────────────

bool BMx280_PIO::beginPIO(PIO pio) {
    if (!_init||_transport!=TRANSPORT_PIO||!_pio_i2c) return false;
    return _pio_i2c->beginPIO(pio);
}

bool BMx280_PIO::beginAutoScan(uint32_t period_ms) {
    if (!_pio_i2c||!_pio_i2c->isPIOActive()) return false;
    setMode(BME280_MODE_NORMAL);
    if (period_ms >= 1000) _standby = BME280_STANDBY_1000MS;
    else if (period_ms >= 500) _standby = BME280_STANDBY_500MS;
    else _standby = BME280_STANDBY_250MS;
    _applyConfig();
    return _pio_i2c->beginAutoScan(_addr, BME280_REG_PRESS_MSB, _raw_async, 8, period_ms);
}

void BMx280_PIO::stopAutoScan() {
    if (!_pio_i2c) return;
    _pio_i2c->stopAutoScan();
    setMode(BME280_MODE_SLEEP);
}

void BMx280_PIO::readAllAsync(float *t, float *p, float *h) {
    if (!_init) { if(t)*t=NAN; if(p)*p=NAN; if(h)*h=NAN; return; }

    // ─── Extract 8 data bytes from DMA ring buffer ───────────────────
    // CH2 uses ring size 5 (32 bytes = 8 words). The 11-word burst
    // (3 ACKs + 8 data bytes) wraps: data bytes 5,6,7 land at
    // positions 0,1,2 while data bytes 0..4 stay at positions 3..7.
    //
    // Layout after burst (ring-wrapped):
    //   _raw_async[0]=data5, [1]=data6, [2]=data7,
    //   _raw_async[3]=data0, [4]=data1, [5]=data2, [6]=data3, [7]=data4
    //
    // Note: For continuous multi-burst operation, the start offset
    // shifts each burst. A PIO IRQ handler should track the ring
    // write pointer for robust synchronization.
    _raw_bytes[0] = (rev8(_raw_async[3] & 0xFF) >> 1) & 0xFF;  // data byte 0
    _raw_bytes[1] = (rev8(_raw_async[4] & 0xFF) >> 1) & 0xFF;  // data byte 1
    _raw_bytes[2] = (rev8(_raw_async[5] & 0xFF) >> 1) & 0xFF;  // data byte 2
    _raw_bytes[3] = (rev8(_raw_async[6] & 0xFF) >> 1) & 0xFF;  // data byte 3
    _raw_bytes[4] = (rev8(_raw_async[7] & 0xFF) >> 1) & 0xFF;  // data byte 4
    _raw_bytes[5] = (rev8(_raw_async[0] & 0xFF) >> 1) & 0xFF;  // data byte 5 (wrapped)
    _raw_bytes[6] = (rev8(_raw_async[1] & 0xFF) >> 1) & 0xFF;  // data byte 6 (wrapped)
    _raw_bytes[7] = (rev8(_raw_async[2] & 0xFF) >> 1) & 0xFF;  // data byte 7 (wrapped)

    int32_t aP = ((int32_t)_raw_bytes[0]<<12)|((int32_t)_raw_bytes[1]<<4)|(_raw_bytes[2]>>4);
    int32_t aT = ((int32_t)_raw_bytes[3]<<12)|((int32_t)_raw_bytes[4]<<4)|(_raw_bytes[5]>>4);
    int32_t aH = _is_bme ? (((int32_t)_raw_bytes[6]<<8)|_raw_bytes[7]) : 0;

    int32_t tv1=((((aT>>3)-((int32_t)_T1<<1)))*_T2)>>11;
    int32_t tv2=(((((aT>>4)-_T1)*((aT>>4)-_T1))>>12)*_T3)>>14;
    _t_fine=tv1+tv2;
    if(t)*t=(float)((_t_fine*5+128)>>8)/100.0f;
    if(p){
        double pv1=(double)_t_fine/2.0-64000.0;
        double pv2=pv1*pv1*(double)_P6/32768.0;
        pv2+=pv1*(double)_P5*2.0;
        pv2=pv2/4.0+(double)_P4*65536.0;
        pv1=((double)_P3*pv1*pv1/524288.0+(double)_P2*pv1)/524288.0;
        pv1=(1.0+pv1/32768.0)*(double)_P1;
        if(pv1==0.0){*p=0.0f;}else{
            double pp=1048576.0-(double)aP;
            pp=(pp-pv2/4096.0)*6250.0/pv1;
            pv1=(double)_P9*pp*pp/2147483648.0;
            pv2=pp*(double)_P8/32768.0;
            pp+=(pv1+pv2+(double)_P7)/16.0;
            *p=(float)(pp/100.0);
        }
    }
    if(h){
        if(!_is_bme){*h=0.0f;}else{
            int32_t hv=_t_fine-76800;
            hv=(int32_t)((((((int64_t)aH<<14)-((int64_t)_H4<<20)-((int64_t)_H5*hv))+16384)>>15)*
                         ((((((((int64_t)hv*_H6)>>10)*((((int64_t)hv*_H3)>>11)+32768))>>10)+2097152)*_H2+8192)>>14));
            hv=hv-(((((hv>>15)*(hv>>15))>>7)*(int32_t)_H1)>>4);
            hv=hv<0?0:hv;hv=hv>419430400?419430400:hv;
            *h=(float)((uint32_t)(hv>>12))/1024.0f;
        }
    }
}

uint8_t BMx280_PIO::getChipID() { uint8_t v=0;_i2c_read(BME280_REG_CHIP_ID,&v,1);return v; }
