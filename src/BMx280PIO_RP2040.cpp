#include "BMx280PIO_RP2040.h"
#include "WirePIO.h"
#include <hardware/gpio.h>

BMx280PIO_RP2040::BMx280PIO_RP2040(TwoWire &wire, uint8_t addr)
    : _wire(&wire), _wirepio(nullptr), _addr(addr), _sda(0), _scl(0), _freq(100000),
      _init(false), _is_bme(false),
      _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X),
      _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP) {}

BMx280PIO_RP2040::BMx280PIO_RP2040(uint8_t sda, uint8_t scl, uint8_t addr, uint32_t freq, PIO pio)
    : _wire(nullptr), _wirepio(new WirePIO(sda, scl, freq, pio)),
      _addr(addr), _sda(sda), _scl(scl), _freq(freq),
      _init(false), _is_bme(false),
      _osrs_t(BME280_OS_1X), _osrs_p(BME280_OS_1X), _osrs_h(BME280_OS_1X),
      _filter(BME280_FILTER_OFF), _standby(BME280_STANDBY_250MS), _mode(BME280_MODE_SLEEP) {}

BMx280PIO_RP2040::~BMx280PIO_RP2040() {
    if (_wirepio) { _wirepio->end(); delete _wirepio; }
}

static inline void dly(uint32_t f) { delayMicroseconds(500000 / f < 2 ? 2 : 500000 / f); }
static inline void sl(uint p)    { gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0); }
static inline void sh(uint p)    { gpio_set_dir(p, GPIO_IN); }
static inline bool sr(uint p)    { return gpio_get(p); }
static inline void cl(uint p)    { gpio_put(p, 0); }
static inline void ch(uint p)    { gpio_put(p, 1); }
static void start(uint sd, uint sc, uint32_t f) { sl(sd); dly(f); cl(sc); dly(f); }
static void stop(uint sd, uint sc, uint32_t f)  { sl(sd); dly(f); ch(sc); dly(f); sh(sd); dly(f); }
static bool wr(uint sd, uint sc, uint32_t f, uint8_t b) {
    for (uint8_t m = 0x80; m; m >>= 1) { if (b & m) sh(sd); else sl(sd); dly(f); ch(sc); dly(f); cl(sc); }
    sh(sd); dly(f); ch(sc); dly(f); bool a = !sr(sd); cl(sc); dly(f); return a;
}
static uint8_t rd(uint sd, uint sc, uint32_t f, bool last) {
    uint8_t v = 0; sh(sd);
    for (int i = 0; i < 8; i++) { ch(sc); dly(f); v = (v << 1) | (sr(sd) ? 1 : 0); cl(sc); dly(f); }
    last ? sh(sd) : sl(sd); dly(f); ch(sc); dly(f); cl(sc); dly(f); sh(sd); return v;
}

bool BMx280PIO_RP2040::_i2c_write(uint8_t reg, const uint8_t *data, size_t len) {
    if (_wirepio) {
        _wirepio->beginTransmission(_addr); _wirepio->write(reg);
        for (size_t i = 0; i < len; i++) _wirepio->write(data[i]);
        return _wirepio->endTransmission() == 0;
    } else {
        _wire->beginTransmission(_addr); _wire->write(reg);
        for (size_t i = 0; i < len; i++) _wire->write(data[i]);
        return _wire->endTransmission() == 0;
    }
}

bool BMx280PIO_RP2040::_i2c_read(uint8_t reg, uint8_t *data, size_t len) {    if (_wirepio) {
        // Fast path: PIO+DMA burstRead for up to 8 bytes
        if (len <= 8 && _wirepio->burstRead(_addr, reg, data, len) == len)
            return true;
        // GPIO fallback for longer reads or if burstRead unavailable
        uint8_t sd = _sda, sc = _scl; uint32_t f = _freq;
        for (size_t i = 0; i < len; i++) {
            gpio_init(sd); gpio_set_dir(sd, GPIO_IN); gpio_pull_up(sd);
            gpio_init(sc); gpio_set_dir(sc, GPIO_OUT); gpio_put(sc, 1);
            start(sd, sc, f);
            if (!wr(sd, sc, f, (uint8_t)(_addr << 1))) { stop(sd, sc, f); return false; }
            if (!wr(sd, sc, f, (uint8_t)(reg + i))) { stop(sd, sc, f); return false; }
            stop(sd, sc, f);
            start(sd, sc, f);
            if (!wr(sd, sc, f, (uint8_t)((_addr << 1) | 1))) { stop(sd, sc, f); return false; }
            data[i] = rd(sd, sc, f, true);
            stop(sd, sc, f);
        }
        return true;
    } else {
        _wire->beginTransmission(_addr); _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        size_t n = _wire->requestFrom(_addr, len);
        for (size_t i = 0; i < n && i < len; i++) data[i] = _wire->read();
        return n == len;
    }
}

bool BMx280PIO_RP2040::begin() {
    if (_init) return true;
    if (_wirepio) {
        _wirepio->begin();
        if (!_wirepio->isRunning()) return false;
    }
    uint8_t rst = BME280_RESET_VALUE;
    _i2c_write(BME280_REG_RESET, &rst, 1);
    sleep_ms(15);
    uint8_t cid = 0;
    if (!_i2c_read(BME280_REG_CHIP_ID, &cid, 1)) return false;
    _is_bme = (cid == BME280_CHIP_ID_VALUE);
    if (cid != BMP280_CHIP_ID && cid != BME280_CHIP_ID_VALUE) return false;
    if (!_loadCalibration()) { Serial.println("CAL FAIL"); return false; }

    _applyConfig();
    _init = true; return true;
}

bool BMx280PIO_RP2040::beginPIO(PIO pio) {
    if (!_wirepio || !_init) return false;
    _wirepio->end(); _wirepio->setPIO(pio); _wirepio->begin();
    return _wirepio->isRunning();
}

bool BMx280PIO_RP2040::_loadCalibration() {
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

void BMx280PIO_RP2040::_applyConfig() {
    uint8_t d;
    if (_is_bme) { d = _osrs_h & 0x07; _i2c_write(BME280_REG_CTRL_HUM, &d, 1); }
    d = ((_osrs_t & 0x07) << 5) | ((_osrs_p & 0x07) << 2) | (_mode & 0x03);
    _i2c_write(BME280_REG_CTRL_MEAS, &d, 1);
    d = ((_standby & 0x07) << 5) | ((_filter & 0x07) << 2);
    _i2c_write(BME280_REG_CONFIG, &d, 1);
}

void BMx280PIO_RP2040::setMode(uint8_t mode) {
    _mode = mode & 0x03; if (!_init) return;
    uint8_t d; _i2c_read(BME280_REG_CTRL_MEAS, &d, 1);
    d = (d & 0xFC) | (_mode & 0x03); _i2c_write(BME280_REG_CTRL_MEAS, &d, 1);
}

uint8_t BMx280PIO_RP2040::readRegister(uint8_t r) { uint8_t v=0; _i2c_read(r,&v,1); return v; }
void BMx280PIO_RP2040::writeRegister(uint8_t r, uint8_t v) { _i2c_write(r,&v,1); }
void BMx280PIO_RP2040::readRegisters(uint8_t r, uint8_t *d, size_t n) { _i2c_read(r,d,n); }

uint8_t BMx280PIO_RP2040::_measTime() {
    const uint8_t m[] = {0,1,2,4,8,16};
    uint32_t t = 1250 + 2300*m[_osrs_t&7] + 2300*m[_osrs_p&7] + 575;
    if (_is_bme) t += 2300*m[_osrs_h&7] + 575;
    return (uint8_t)((t+1500)/1000);
}

bool BMx280PIO_RP2040::takeForcedMeasurement() {
    if (!_init) return false; setMode(BME280_MODE_FORCED); sleep_ms(_measTime());
    uint32_t start = millis(); uint8_t st;
    do { if (!_i2c_read(BME280_REG_STATUS,&st,1)) return false;
         if (millis()-start>100) return false; sleep_ms(1); } while (st&0x08);
    return true;
}

void BMx280PIO_RP2040::_readRaw(int32_t *t, int32_t *p, int32_t *h) {
    uint8_t d[8]; _i2c_read(BME280_REG_PRESS_MSB, d, 8);
    if(p)*p=((int32_t)d[0]<<12)|((int32_t)d[1]<<4)|(d[2]>>4);
    if(t)*t=((int32_t)d[3]<<12)|((int32_t)d[4]<<4)|(d[5]>>4);
    if(h&&_is_bme)*h=((int32_t)d[6]<<8)|d[7]; else if(h)*h=0;
}

float BMx280PIO_RP2040::readTemperature() {
    if(!_init)return NAN; int32_t aT; _readRaw(&aT,0,0);
    int32_t v1=((((aT>>3)-((int32_t)_T1<<1)))*_T2)>>11;
    int32_t v2=(((((aT>>4)-_T1)*((aT>>4)-_T1))>>12)*_T3)>>14;
    _t_fine=v1+v2; return(float)((_t_fine*5+128)>>8)/100.0f;
}

float BMx280PIO_RP2040::readPressure() {
    if(!_init)return NAN; int32_t aT,aP; _readRaw(&aT,&aP,0); readTemperature();
    double v1=(double)_t_fine/2.0-64000.0,v2=v1*v1*(double)_P6/32768.0;
    v2+=v1*(double)_P5*2.0; v2=v2/4.0+(double)_P4*65536.0;
    v1=((double)_P3*v1*v1/524288.0+(double)_P2*v1)/524288.0;
    v1=(1.0+v1/32768.0)*(double)_P1; if(v1==0.0)return 0.0f;
    double p=1048576.0-(double)aP; p=(p-v2/4096.0)*6250.0/v1;
    v1=(double)_P9*p*p/2147483648.0; v2=p*(double)_P8/32768.0;
    p+=(v1+v2+(double)_P7)/16.0; return(float)(p/100.0);
}

float BMx280PIO_RP2040::readHumidity() {
    if(!_init||!_is_bme)return 0.0f; int32_t aT,aH; _readRaw(&aT,0,&aH); readTemperature();
    int32_t hv=_t_fine-76800;
    hv=(int32_t)((((((int64_t)aH<<14)-((int64_t)_H4<<20)-((int64_t)_H5*hv))+16384)>>15)*
                 ((((((((int64_t)hv*_H6)>>10)*((((int64_t)hv*_H3)>>11)+32768))>>10)+2097152)*_H2+8192)>>14));
    hv=hv-(((((hv>>15)*(hv>>15))>>7)*(int32_t)_H1)>>4);
    hv=hv<0?0:hv; hv=hv>419430400?419430400:hv; return(float)((uint32_t)(hv>>12))/1024.0f;
}

void BMx280PIO_RP2040::readAll(float *t, float *p, float *h) {
    if(!_init){if(t)*t=NAN;if(p)*p=NAN;if(h)*h=NAN;return;}
    int32_t aT,aP,aH; _readRaw(&aT,&aP,&aH);
    int32_t tv1=((((aT>>3)-((int32_t)_T1<<1)))*_T2)>>11;
    int32_t tv2=(((((aT>>4)-_T1)*((aT>>4)-_T1))>>12)*_T3)>>14;
    _t_fine=tv1+tv2; if(t)*t=(float)((_t_fine*5+128)>>8)/100.0f;
    if(p){double pv1=(double)_t_fine/2.0-64000.0,pv2=pv1*pv1*(double)_P6/32768.0;
           pv2+=pv1*(double)_P5*2.0; pv2=pv2/4.0+(double)_P4*65536.0;
           pv1=((double)_P3*pv1*pv1/524288.0+(double)_P2*pv1)/524288.0;
           pv1=(1.0+pv1/32768.0)*(double)_P1; if(pv1==0.0){*p=0.0f;}else{
           double pp=1048576.0-(double)aP; pp=(pp-pv2/4096.0)*6250.0/pv1;
           pv1=(double)_P9*pp*pp/2147483648.0; pv2=pp*(double)_P8/32768.0;
           pp+=(pv1+pv2+(double)_P7)/16.0; *p=(float)(pp/100.0);}}
    if(h){if(!_is_bme){*h=0.0f;}else{int32_t hv=_t_fine-76800;
           hv=(int32_t)((((((int64_t)aH<<14)-((int64_t)_H4<<20)-((int64_t)_H5*hv))+16384)>>15)*
                        ((((((((int64_t)hv*_H6)>>10)*((((int64_t)hv*_H3)>>11)+32768))>>10)+2097152)*_H2+8192)>>14));
           hv=hv-(((((hv>>15)*(hv>>15))>>7)*(int32_t)_H1)>>4);
           hv=hv<0?0:hv; hv=hv>419430400?419430400:hv; *h=(float)((uint32_t)(hv>>12))/1024.0f;}}
}

uint8_t BMx280PIO_RP2040::getChipID() { uint8_t v=0; _i2c_read(BME280_REG_CHIP_ID,&v,1); return v; }
