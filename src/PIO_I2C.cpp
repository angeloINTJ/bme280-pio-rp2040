#include "PIO_I2C.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include "i2c.pio.h"

static uint8_t rev8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}
static inline uint16_t mk_cmd(bool s, bool r, bool p, uint8_t d) {
    uint8_t inv = (~rev8(d)) & 0xFF;
    return (s?1:0) | ((r?1:0)<<1) | (((uint16_t)inv)<<2) | ((p?1:0)<<10);
}

PIO_I2C::PIO_I2C(uint8_t sda, uint8_t scl, uint32_t freq)
    : _sda(sda), _scl(scl), _freq(freq), _pio(nullptr), _sm(-1), _offset(-1),
      _initialized(false), _pio_active(false), _auto_scan(false),
      _dma_tx_chan(-1), _dma_rx_chan(-1), _dma_ctrl_chan(-1),
      _pwm_slice(-1), _burst_len(0), _burst_buf(nullptr), _cmd_count(0) {}
PIO_I2C::~PIO_I2C() { end(); }

bool PIO_I2C::begin() {
    if (_initialized) return true;
    gpio_init(_sda); gpio_set_dir(_sda, GPIO_IN); gpio_pull_up(_sda);
    gpio_init(_scl); gpio_set_dir(_scl, GPIO_OUT); gpio_put(_scl, 1);
    _initialized = true; return true;
}
void PIO_I2C::end() {
    if (_auto_scan) stopAutoScan();
    if (_pio_active) {
        if (_dma_tx_chan>=0){dma_channel_unclaim(_dma_tx_chan);_dma_tx_chan=-1;}
        if (_dma_rx_chan>=0){dma_channel_unclaim(_dma_rx_chan);_dma_rx_chan=-1;}
        if (_dma_ctrl_chan>=0){dma_channel_unclaim(_dma_ctrl_chan);_dma_ctrl_chan=-1;}
        if (_pwm_slice>=0){pwm_set_enabled(_pwm_slice,false);}
        if (_pio&&_offset>=0){pio_sm_set_enabled(_pio,_sm,false);pio_sm_unclaim(_pio,_sm);pio_remove_program(_pio,&i2c_master_program,_offset);}
        _pio_active=false;
    }
    if(_initialized){gpio_set_dir(_sda,GPIO_IN);gpio_disable_pulls(_sda);gpio_set_dir(_scl,GPIO_IN);gpio_disable_pulls(_scl);_initialized=false;}
}
static inline void sda_lo(uint p){gpio_set_dir(p,GPIO_OUT);gpio_put(p,0);}
static inline void sda_hi(uint p){gpio_set_dir(p,GPIO_IN);}
static inline bool sda_read(uint p){return gpio_get(p);}
static inline void scl_lo(uint p){gpio_put(p,0);}
static inline void scl_hi(uint p){gpio_put(p,1);}
static void i2c_delay(uint32_t f){delayMicroseconds(500000/f<2?2:500000/f);}
static bool i2c_write_byte(uint sd,uint sc,uint32_t f,uint8_t d){
    for(uint8_t m=0x80;m;m>>=1){if(d&m)sda_hi(sd);else sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);scl_lo(sc);}
    sda_hi(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);bool n=sda_read(sd);scl_lo(sc);i2c_delay(f);return !n;
}
static uint8_t i2c_read_byte(uint sd,uint sc,uint32_t f,bool last){
    uint8_t d=0;sda_hi(sd);
    for(int i=0;i<8;i++){scl_hi(sc);i2c_delay(f);d=(d<<1)|(sda_read(sd)?1:0);scl_lo(sc);i2c_delay(f);}
    if(last)sda_hi(sd);else sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);scl_lo(sc);i2c_delay(f);sda_hi(sd);return d;
}
static void i2c_start(uint sd,uint sc,uint32_t f){sda_lo(sd);i2c_delay(f);scl_lo(sc);i2c_delay(f);}
static void i2c_stop(uint sd,uint sc,uint32_t f){sda_lo(sd);i2c_delay(f);scl_hi(sc);i2c_delay(f);sda_hi(sd);i2c_delay(f);}
bool PIO_I2C::_sendAddress(uint8_t a,bool r){i2c_start(_sda,_scl,_freq);return i2c_write_byte(_sda,_scl,_freq,(a<<1)|(r?1:0));}
void PIO_I2C::_sendStop(){i2c_stop(_sda,_scl,_freq);}
void PIO_I2C::_waitIdle(){}
bool PIO_I2C::write(uint8_t a,const uint8_t*d,size_t n,bool s){
    if(!_initialized||n==0)return false;
    i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1))){i2c_stop(_sda,_scl,_freq);return false;}
    for(size_t i=0;i<n;i++){if(!i2c_write_byte(_sda,_scl,_freq,d[i])){i2c_stop(_sda,_scl,_freq);return false;}}
    if(s)i2c_stop(_sda,_scl,_freq);return true;
}
bool PIO_I2C::read(uint8_t a,uint8_t*d,size_t n,bool s){
    if(!_initialized||n==0)return false;
    i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1)|1)){i2c_stop(_sda,_scl,_freq);return false;}
    for(size_t i=0;i<n;i++)d[i]=i2c_read_byte(_sda,_scl,_freq,(i==n-1));
    if(s)i2c_stop(_sda,_scl,_freq);return true;
}
bool PIO_I2C::writeThenRead(uint8_t a,const uint8_t*wd,size_t wl,uint8_t*rd,size_t rl){
    if(!_initialized)return false;
    if(wl>0){i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1))){i2c_stop(_sda,_scl,_freq);return false;}for(size_t i=0;i<wl;i++){if(!i2c_write_byte(_sda,_scl,_freq,wd[i])){i2c_stop(_sda,_scl,_freq);return false;}}i2c_stop(_sda,_scl,_freq);}
    if(rl>0){i2c_start(_sda,_scl,_freq);if(!i2c_write_byte(_sda,_scl,_freq,(a<<1)|1)){i2c_stop(_sda,_scl,_freq);return false;}for(size_t i=0;i<rl;i++)rd[i]=i2c_read_byte(_sda,_scl,_freq,(i==rl-1));i2c_stop(_sda,_scl,_freq);}
    return true;
}
void PIO_I2C::scan(){
    if(!_initialized)return;Serial.println("I2C Scan:");int f=0;
    for(int a=1;a<0x78;a++){i2c_start(_sda,_scl,_freq);bool ack=i2c_write_byte(_sda,_scl,_freq,(a<<1));i2c_stop(_sda,_scl,_freq);if(ack){Serial.print("0x");Serial.print(a,HEX);Serial.print(" ");f++;}}
    Serial.print("(");Serial.print(f);Serial.println(" devices)");
}

void PIO_I2C::_buildBurstCommands(uint8_t addr, uint8_t reg, size_t len) {
    _cmd_count = 0;
    _cmd_buf[_cmd_count++] = mk_cmd(true, false, false, addr << 1);
    _cmd_buf[_cmd_count++] = mk_cmd(false, false, false, reg);
    _cmd_buf[_cmd_count++] = mk_cmd(true, false, false, (addr << 1) | 1);
    for (size_t i = 0; i < len - 1; i++)
        _cmd_buf[_cmd_count++] = mk_cmd(false, true, false, 0xFF);
    _cmd_buf[_cmd_count++] = mk_cmd(false, true, true, 0xFF);
}

void PIO_I2C::_setupPWM(uint32_t period_ms) {
    _pwm_slice = 7;
    uint32_t sys_clk = clock_get_hz(clk_sys);
    float target_hz = 1000.0f / (float)period_ms;
    float div = 1.0f;
    uint32_t wrap = (uint32_t)((float)sys_clk / (div * target_hz)) - 1;
    while (wrap > 65535 && div < 256.0f) { div *= 2.0f; wrap = (uint32_t)((float)sys_clk / (div * target_hz)) - 1; }
    if (wrap > 65535) wrap = 65535;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_init(_pwm_slice, &cfg, false);
}

void PIO_I2C::_setupDMA() {
    dma_channel_config c_rx = dma_channel_get_default_config(_dma_rx_chan);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_32);
    channel_config_set_read_increment(&c_rx, false);
    channel_config_set_write_increment(&c_rx, true);
    channel_config_set_ring(&c_rx, true, 5);
    channel_config_set_dreq(&c_rx, pio_get_dreq(_pio, _sm, false));
    dma_channel_configure(_dma_rx_chan, &c_rx, _burst_buf, &_pio->rxf[_sm], 0xFFFFFFFF, true);

    dma_channel_config c_tx = dma_channel_get_default_config(_dma_tx_chan);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_32);
    channel_config_set_read_increment(&c_tx, true);
    channel_config_set_write_increment(&c_tx, false);
    channel_config_set_dreq(&c_tx, pio_get_dreq(_pio, _sm, true));
    dma_channel_configure(_dma_tx_chan, &c_tx, &_pio->txf[_sm], _cmd_buf, _cmd_count, false);

    _ctrl_data[0] = _cmd_count;
    _ctrl_data[1] = (uint32_t)_cmd_buf;
    dma_channel_config c_ctrl = dma_channel_get_default_config(_dma_ctrl_chan);
    channel_config_set_transfer_data_size(&c_ctrl, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl, true);
    channel_config_set_write_increment(&c_ctrl, true);
    channel_config_set_ring(&c_ctrl, false, 3);
    channel_config_set_ring(&c_ctrl, true, 3);
    channel_config_set_dreq(&c_ctrl, DREQ_PWM_WRAP0 + _pwm_slice);
    dma_channel_configure(_dma_ctrl_chan, &c_ctrl, &dma_hw->ch[_dma_tx_chan].al3_transfer_count, _ctrl_data, 0xFFFFFFFF, false);
}

bool PIO_I2C::beginPIO(PIO pio) {
    if (_pio_active) return true;
    if (!_initialized) return false;
    _pio = pio;
    if (!pio_can_add_program(_pio, &i2c_master_program)) return false;
    _offset = pio_add_program(_pio, &i2c_master_program);
    if (_offset < 0) return false;
    int sm = pio_claim_unused_sm(_pio, false);
    if (sm < 0) { pio_remove_program(_pio, &i2c_master_program, _offset); _offset = -1; return false; }
    _sm = sm;

    pio_sm_config c = i2c_master_program_get_default_config(_offset);
    sm_config_set_out_pins(&c, _sda, 1);
    sm_config_set_set_pins(&c, _sda, 1);
    sm_config_set_in_pins(&c, _sda);
    sm_config_set_sideset_pins(&c, _scl);
    sm_config_set_in_shift(&c, false, true, 8);
    float div = (float)clock_get_hz(clk_sys) / ((float)_freq * 13.0f);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(_pio, _sm, _offset, &c);
    pio_sm_set_pindirs_with_mask(_pio, _sm, (1u << _scl), (1u << _sda) | (1u << _scl));

    _dma_tx_chan = dma_claim_unused_channel(false);
    _dma_rx_chan = dma_claim_unused_channel(false);
    _dma_ctrl_chan = dma_claim_unused_channel(false);
    if (_dma_tx_chan < 0 || _dma_rx_chan < 0 || _dma_ctrl_chan < 0) {
        if (_dma_tx_chan >= 0) dma_channel_unclaim(_dma_tx_chan);
        if (_dma_rx_chan >= 0) dma_channel_unclaim(_dma_rx_chan);
        if (_dma_ctrl_chan >= 0) dma_channel_unclaim(_dma_ctrl_chan);
        pio_sm_unclaim(_pio, _sm);
        pio_remove_program(_pio, &i2c_master_program, _offset);
        _offset = -1; return false;
    }
    _pio_active = true; return true;
}

bool PIO_I2C::beginAutoScan(uint8_t addr, uint8_t reg, uint32_t *buf, size_t len, uint32_t period_ms) {
    if (!_pio_active) return false;
    _burst_len = len; _burst_buf = buf;
    _buildBurstCommands(addr, reg, len);
    _setupPWM(period_ms);
    gpio_pull_up(_sda);
    gpio_set_function(_sda, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(_scl, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    _setupDMA();
    pio_sm_set_enabled(_pio, _sm, true);
    dma_start_channel_mask(1u << _dma_ctrl_chan);
    pwm_set_enabled(_pwm_slice, true);
    _auto_scan = true; return true;
}

void PIO_I2C::stopAutoScan() {
    if (!_auto_scan) return;
    pwm_set_enabled(_pwm_slice, false);
    dma_channel_abort(_dma_ctrl_chan);
    dma_channel_abort(_dma_tx_chan);
    dma_channel_abort(_dma_rx_chan);
    pio_sm_set_enabled(_pio, _sm, false);
    gpio_set_function(_sda, GPIO_FUNC_SIO);
    gpio_set_function(_scl, GPIO_FUNC_SIO);
    gpio_set_dir(_sda, GPIO_IN); gpio_pull_up(_sda);
    gpio_set_dir(_scl, GPIO_OUT); gpio_put(_scl, 1);
    _auto_scan = false;
}

void PIO_I2C::extractBytes(const uint32_t *src, uint8_t *dst, size_t len) {
    for (size_t i = 0; i < len; i++) dst[i] = src[i] & 0xFF;
}
