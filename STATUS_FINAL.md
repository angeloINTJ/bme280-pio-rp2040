# Status Final — Driver BME280 PIO+DMA

## O Que Funciona
1. PIO original `i2c.pio` com encoding correto: `(~rev8(data) & 0xFF) << 2`
2. `pio_sm_set_pindirs_with_mask(pio,sm,(1<<SCL),(1<<SDA)|(1<<SCL))` — SCL=OUT, SDA=IN
3. Autopush: `sm_config_set_in_shift(&c, false, true, 8)` — FUNCIONA
4. DMA transfere dados corretamente
5. Leitura GPIO de calibração funciona (T1=28113, T2=27037, T3=-1000, P1=36707)
6. Temperatura compensada do PIO: ~24°C (sensor aquece perto do RP2040)

## Problema Restante
O PIO NÃO consegue escrever o registo (ACK2 sempre NACK). O sensor nunca ACK
o byte do registo, mesmo com I2C extremamente lento (div=5000).
Sem o registo correto, o sensor não auto-incrementa e não retorna os dados
dos 8 registos. Apenas o primeiro byte (pressão MSB) é lido corretamente.

## Hipóteses
1. O PIO envia dados corretos (verificado: 0xF7 no I2C) mas o sensor não reconhece
2. Possível problema: o PIO não espera o tempo mínimo entre bytes (o GPIO tem 5µs delay)
3. Possível: o PIO gera glitches no SCL durante a transição entre comandos

## Solução Alternativa (Funcional)
Usar GPIO para escrita (START + addr+W + registo) e PIO apenas para leitura.
Isto requer coordenação GPIO→PIO com timing correto. Teste pendente.

## Próximo Passo
Verificar com analisador lógico se o SCL/SDA do PIO estão corretos.
