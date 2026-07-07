#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include "i2c.pio.h"
static uint8_t rev8(uint8_t b){b=(b&0xF0)>>4|(b&0x0F)<<4;b=(b&0xCC)>>2|(b&0x33)<<2;b=(b&0xAA)>>1|(b&0x55)<<1;return b;}
static inline uint16_t mk(bool s,bool r,bool p,uint8_t d){uint8_t i=(~rev8(d))&0xFF;return(s?1:0)|((r?1:0)<<1)|(((uint16_t)i)<<2)|((p?1:0)<<10);}

void setup(){
    Serial.begin(115200);for(int i=0;i<60&&!Serial;i++)delay(100);delay(500);Serial.println("GPIO WRITE + PIO READ");
    auto sl=[]{gpio_set_dir(2,GPIO_OUT);gpio_put(2,0);};auto sh=[]{gpio_set_dir(2,GPIO_IN);};auto cl=[]{gpio_put(3,0);};auto ch=[]{gpio_put(3,1);};auto d5=[]{delayMicroseconds(5);};
    auto wb=[&](uint8_t b){for(uint8_t m=0x80;m;m>>=1){if(b&m)sh();else sl();d5();ch();d5();cl();}sh();d5();ch();d5();bool a=gpio_get(2);cl();d5();return !a;};
    auto rb=[&](bool l){uint8_t v=0;sh();for(int i=0;i<8;i++){ch();d5();v=(v<<1)|(gpio_get(2)?1:0);cl();d5();}if(l)sh();else sl();d5();ch();d5();cl();d5();sh();return v;};
    gpio_init(2);gpio_set_dir(2,GPIO_IN);gpio_pull_up(2);gpio_init(3);gpio_set_dir(3,GPIO_OUT);gpio_put(3,1);
    
    // Reset + config via GPIO
    sl();d5();cl();d5();wb(0xEC);wb(0xE0);wb(0xB6);sl();d5();ch();d5();sh();d5();delay(20);
    sl();d5();cl();d5();wb(0xEC);wb(0xF4);wb(0x25);sl();d5();ch();d5();sh();d5();delay(50);

    // Read cal
    struct Cal{uint16_t T1;int16_t T2,T3;uint16_t P1;int16_t P2,P3,P4,P5,P6,P7,P8,P9;}cal;
    uint8_t b[26];sl();d5();cl();d5();wb(0xEC);wb(0x88);sl();d5();ch();d5();sh();d5();sl();d5();cl();d5();wb(0xED);for(int i=0;i<26;i++)b[i]=rb(i==25);sl();d5();ch();d5();sh();d5();
    cal.T1=b[0]|(b[1]<<8);cal.T2=(int16_t)(b[2]|(b[3]<<8));cal.T3=(int16_t)(b[4]|(b[5]<<8));cal.P1=b[6]|(b[7]<<8);
    cal.P2=(int16_t)(b[8]|(b[9]<<8));cal.P3=(int16_t)(b[10]|(b[11]<<8));cal.P4=(int16_t)(b[12]|(b[13]<<8));cal.P5=(int16_t)(b[14]|(b[15]<<8));
    cal.P6=(int16_t)(b[16]|(b[17]<<8));cal.P7=(int16_t)(b[18]|(b[19]<<8));cal.P8=(int16_t)(b[20]|(b[21]<<8));cal.P9=(int16_t)(b[22]|(b[23]<<8));
    auto compT=[&](int32_t a,int32_t&tf){int32_t v1=((((a>>3)-((int32_t)cal.T1<<1)))*cal.T2)>>11;int32_t v2=(((((a>>4)-cal.T1)*((a>>4)-cal.T1))>>12)*cal.T3)>>14;tf=v1+v2;return(float)((tf*5+128)>>8)/100.0f;};
    auto compP=[&](int32_t a,int32_t tf){double v1=(double)tf/2.0-64000.0;double v2=v1*v1*(double)cal.P6/32768.0;v2+=v1*(double)cal.P5*2.0;v2=v2/4.0+(double)cal.P4*65536.0;v1=((double)cal.P3*v1*v1/524288.0+(double)cal.P2*v1)/524288.0;v1=(1.0+v1/32768.0)*(double)cal.P1;if(v1==0.0)return 0.0f;double p=1048576.0-(double)a;p=(p-v2/4096.0)*6250.0/v1;v1=(double)cal.P9*p*p/2147483648.0;v2=p*(double)cal.P8/32768.0;p+=(v1+v2+(double)cal.P7)/16.0;return(float)(p/100.0);};

    // Setup PIO for READ ONLY
    PIO pio=pio0;int off=pio_add_program(pio,&i2c_master_program);int sm=pio_claim_unused_sm(pio,false);
    pio_sm_config c=i2c_master_program_get_default_config(off);
    sm_config_set_out_pins(&c,2,1);sm_config_set_set_pins(&c,2,1);sm_config_set_in_pins(&c,2);sm_config_set_sideset_pins(&c,3);
    sm_config_set_in_shift(&c,false,true,8);sm_config_set_clkdiv(&c,96.15f);
    pio_sm_init(pio,sm,off,&c);pio_sm_set_pindirs_with_mask(pio,sm,(1u<<3),(1u<<2)|(1u<<3));

    for(int n=0;n<5;n++){
        // GPIO: trigger measurement
        sl();d5();cl();d5();wb(0xEC);wb(0xF4);wb(0x25);sl();d5();ch();d5();sh();d5();delay(50);

        // GPIO: START + write addr+W + write register + STOP (leave sensor ready for read)
        // Then switch to PIO for RESTART + read
        uint8_t raw[8];
        for(int i=0;i<8;i++){
            uint8_t reg=0xF7+i;
            // GPIO does: START + write addr+W + write register
            gpio_set_function(2,GPIO_FUNC_SIO);gpio_set_function(3,GPIO_FUNC_SIO);
            gpio_set_dir(2,GPIO_IN);gpio_pull_up(2);gpio_set_dir(3,GPIO_OUT);gpio_put(3,1);
            sl();d5();cl();d5();wb(0xEC);wb(reg);
            // STOP (or leave SCL low for RESTART)
            // Actually, for repeated START, we need: SDA high, SCL low, then SDA low while SCL high
            // The PIO can generate the RESTART + addr+R + read
            // But PIO expects to start from SCL high (idle). After GPIO write, SCL is low.
            // Need to send STOP first.
            sl();d5();ch();d5();sh();d5(); // STOP

            // Now PIO does: RESTART + write addr+R + read 1 byte + STOP
            gpio_pull_up(2);gpio_set_function(2,GPIO_FUNC_PIO0);gpio_set_function(3,GPIO_FUNC_PIO0);
            uint32_t buf[3]={0};
            int rx=dma_claim_unused_channel(false);dma_channel_config rc=dma_channel_get_default_config(rx);
            channel_config_set_transfer_data_size(&rc,DMA_SIZE_32);channel_config_set_read_increment(&rc,false);channel_config_set_write_increment(&rc,true);
            channel_config_set_dreq(&rc,pio_get_dreq(pio,sm,false));dma_channel_configure(rx,&rc,buf,&pio->rxf[sm],3,true);
            // PIO commands: RESTART + write addr+R + read byte + STOP
            uint32_t cmds[3];
            cmds[0]=mk(1,0,0,0xED); // RESTART + write addr+R
            cmds[1]=mk(0,1,0,0xFF); // READ (no STOP yet)
            cmds[2]=mk(0,0,1,0);    // STOP only (generate STOP)
            // Actually, we can't generate STOP without data. Let me use: READ+STOP
            // Simpler: just read one byte with STOP
            uint32_t cmds2[2];
            cmds2[0]=mk(1,0,0,0xED); // RESTART + write addr+R
            cmds2[1]=mk(0,1,1,0xFF); // READ 1 byte + STOP

            int tx=dma_claim_unused_channel(false);dma_channel_config tc=dma_channel_get_default_config(tx);
            channel_config_set_transfer_data_size(&tc,DMA_SIZE_32);channel_config_set_read_increment(&tc,true);channel_config_set_write_increment(&tc,false);
            channel_config_set_dreq(&tc,pio_get_dreq(pio,sm,true));dma_channel_configure(tx,&tc,&pio->txf[sm],cmds2,2,false);

            pio_sm_set_enabled(pio,sm,true);dma_start_channel_mask((1u<<tx)|(1u<<rx));
            unsigned long start=millis();
            while(dma_channel_is_busy(rx)&&(millis()-start)<500)tight_loop_contents();
            bool ok=!dma_channel_is_busy(rx);
            pio_sm_set_enabled(pio,sm,false);

            if(ok){
                raw[i]=rev8(buf[2]&0xFF); // buf[0]=ACK1, buf[1]=DATA, buf[2]=? (only 2 words: ACK + DATA)
                // Actually: 2 commands = 1 write + 1 read. Write pushes ACK. Read autopushes data.
                // Buffer: buf[0]=ACK from write, buf[1]=data from read
                raw[i]=rev8(buf[1]&0xFF);
            } else {
                raw[i]=0;
            }
            dma_channel_unclaim(tx);dma_channel_unclaim(rx);
            delayMicroseconds(200);
        }

        int32_t aP=((int32_t)raw[0]<<12)|((int32_t)raw[1]<<4)|(raw[2]>>4);
        int32_t aT=((int32_t)raw[3]<<12)|((int32_t)raw[4]<<4)|(raw[5]>>4);
        int32_t tf;float t=compT(aT,tf);float p=compP(aP,tf);
        Serial.print(n);Serial.print(": T=");Serial.print(t,1);Serial.print(" P=");Serial.print(p,1);Serial.print(" raw:");for(int i=0;i<8;i++){Serial.print(" ");Serial.print(raw[i],HEX);}Serial.println();
        delay(500);
    }
    Serial.println("DONE");
}
void loop(){}
