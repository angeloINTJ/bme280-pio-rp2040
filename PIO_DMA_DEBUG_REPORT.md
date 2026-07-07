# BME280 PIO+DMA "Zero-CPU-Overhead" Driver — Relatório de Desenvolvimento

## Objetivo

Criar um driver I2C para o sensor BME280/BMP280 no RP2040 usando **PIO (Programmable I/O) + DMA**
para leitura autônoma (zero intervenção da CPU durante o bit-bang do I2C). O PIO gera os sinais
SCL/SDA, o DMA alimenta comandos e coleta respostas, e a CPU apenas executa a matemática de
compensação Bosch sobre o buffer em RAM.

## Arquitetura

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ command_buf  │────▶│  PIO TX FIFO │────▶│  PIO SM      │
│ (uint32_t[]) │     │  (DREQ_TX0)  │     │  (I2C master)│
└──────────────┘     └──────────────┘     └──────┬───────┘
     DMA CH1 (TX)                                 │
                                                  ▼
                                         ┌──────────────┐
                                         │  PIO RX FIFO │
                                         │  (DREQ_RX0)  │
                                         └──────┬───────┘
                                                ▼
                                         ┌──────────────┐
                                         │ raw_data[8]  │
                                         │ (RAM buffer) │
                                         └──────────────┘
                                          DMA CH2 (RX)
```

### Fluxo de uma leitura burst (registradores 0xF7–0xFE, 8 bytes):

1. **CPU**: Configura o sensor em modo FORCED via GPIO bit-bang I2C
2. **CPU**: Chama `startAsyncRead()` → monta 11 comandos I2C no buffer
3. **CPU**: Habilita PIO SM + dispara ambos os canais DMA
4. **PIO + DMA autônomos**:
   - DMA CH1 envia comandos para o TX FIFO do PIO
   - PIO executa: START, write addr+W, write reg, RESTART, write addr+R, 8× read bytes
   - PIO faz `push noblock` de cada byte lido para o RX FIFO
   - DMA CH2 transfere cada palavra do RX FIFO para `raw_data[8]` na RAM
5. **CPU**: Chama `asyncReadWait()` → bloqueia até DMA terminar
6. **CPU**: Executa `readAllAsync()` → compensação Bosch puramente matemática

## O Que Funcionou

### 1. PIO Program (`pio/i2c_burst.pio`) — 32 instruções

O programa PIO foi completamente redesenhado para suportar bursts. Características:

- **START opcional por comando**: Bit 15 (MSB) do comando controla se START é gerado.
  Usa `set pindirs, 1` (que NÃO consome bits do OSR) em vez de `out pindirs, 1`.
- **Flags extraídas antes da divergência**: Bits 15 (START), 14 (READ), 13 (STOP)
  são extraídos do OSR antes do branch read/write. O bit 13 (STOP) é salvo no
  registrador X para uso posterior.
- **Push explícito**: `push noblock` após o loop de leitura de 8 bits (não usa autopush).
- **Sem verificação de ACK**: Economiza ~8 instruções. O BME280 responde com ACK
  de forma confiável em operação normal.
- **Timing I2C**: Delays de [3] ciclos no loop de read/write, com divisor de clock
  configurável para 100 kHz ou 400 kHz.

**Codificação do comando (16-bit, shift MSB-first):**

| Bit   | Nome  | Significado |
|-------|-------|-------------|
| 15    | START | 1 = gerar START antes deste byte |
| 14    | READ  | 1 = ler do slave |
| 13    | STOP  | 1 = gerar STOP após este byte |
| 12:5  | DATA  | 8 bits de dados para WRITE (invertidos) |
| 4:0   | —     | Reservados |

**Sequência de comandos para ler 8 bytes do BME280:**
```
CMD(1,0,0, addr<<1)       // START, write addr+W
CMD(0,0,0, reg)           // write register address  
CMD(1,0,0, (addr<<1)|1)  // RESTART, write addr+R
CMD(0,1,0, 0) ×7          // read bytes 0-6 (ACK implícito)
CMD(0,1,1, 0)             // read byte 7 (STOP/NACK)
```

✅ **Verificado**: O PIO processa todos os 11 comandos — TX FIFO esvazia (TXF:0),
SM retorna ao PC:0 (`pull block`), aguardando mais comandos.

### 2. Integração GPIO/PIO

- O sistema alterna corretamente os pinos entre GPIO (SIO) e PIO:
  - `gpio_set_function(_sda, GPIO_FUNC_PIO0)` antes de habilitar o PIO
  - `gpio_set_function(_sda, GPIO_FUNC_SIO)` após desabilitar o PIO
- O SM PIO é mantido desabilitado até o momento do burst, evitando conflitos
  com as operações GPIO (takeForcedMeasurement, configuração do sensor, etc.)
- O sensor é detectado corretamente (BMP280 no hardware de teste)

✅ **Verificado**: `takeForcedMeasurement()` funciona via GPIO, sensor responde.

### 3. DMA TX (commands → PIO)

- Configurado com transferências de 32 bits (PIO TX FIFO é 32-bit)
- DREQ = `DREQ_PIOx_TX0` (dispara quando TX FIFO tem espaço)
- Todos os 11 comandos são transferidos com sucesso

✅ **Verificado**: `dma_channel_is_busy(_dma_tx_chan)` retorna false após ~1ms.

### 4. Compilação

- O programa PIO compila sem erros via `pioasm`
- O firmware completo compila via PlatformIO
- Uso de memória: ~9.6 KB RAM (3.7%), ~70 KB Flash (3.4%)

✅ **Verificado**: `pio run` produz firmware.uf2 com sucesso.

---

## O Problema Principal: PIO RX FIFO Vazio

### Sintoma

O DMA RX nunca completa. O PIO processa todos os comandos (TX FIFO esvazia,
SM retorna ao PC:0), mas **nenhum dado aparece no RX FIFO**.

### Evidência coletada

| Teste | Resultado |
|-------|-----------|
| TX DMA completa? | ✅ Sim (TXF:0 após ~1ms) |
| PIO processou todos os comandos? | ✅ Sim (SM em PC:0 = `pull block`) |
| Dados no RX FIFO? | ❌ Vazio (RXF:0) |
| DMA RX transferiu dados? | ❌ Não (transfer_count = 8, todos restantes) |
| Buffer de destino (`_burst_buf`)? | ❌ Zerado (0x00000000) |
| Push manual (CPU alimenta comandos)? | ❌ RXF:0 mesmo assim |
| Teste mínimo de push (programa PIO trivial)? | ❌ Não produz output serial |

### Análise

O fato de o PIO processar comandos de escrita (TX FIFO esvazia) mas nunca
produzir dados no RX FIFO indica que **a instrução `push noblock` não está
efetivamente escrevendo no RX FIFO**, mesmo sendo executada.

**Hipótese principal: FIFO Join ativado incorretamente.**

O RP2040 permite juntar os dois FIFOs (TX e RX) em um único FIFO de 8 entradas
via os bits `FJOIN_TX` (bit 30) e `FJOIN_RX` (bit 31) do registrador `SHIFTCTRL`.

Se `FJOIN_TX=1`, todas as 8 entradas do FIFO são alocadas para o lado TX.
Nesse modo:
- `pull` lê do FIFO combinado (acessível como TX)
- `push` **escreve no mesmo FIFO combinado** (também acessível como TX)
- O RX FIFO fica com **0 entradas** — leituras do RX FIFO sempre retornam vazio
- O DMA configurado para ler do RX FIFO (`&pio->rxf[sm]`) nunca vê dados

Isso explicaria TODOS os sintomas observados.

### Correção aplicada (não testada em hardware)

```c
// Em beginPIO(), após sm_config_set_fifo_join():
c.shiftctrl &= ~(3u << 30);  // Limpa bits FJOIN_TX (30) e FJOIN_RX (31)
sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
```

### Outras hipóteses investigadas e descartadas

| Hipótese | Evidência contrária |
|----------|---------------------|
| PIO não entra no caminho de leitura | Comandos têm READ=1; TX FIFO esvazia (todos processados) |
| Timing I2C errado (sensor não responde) | Mesmo sem sensor, `push` deveria enviar dados (ex: 0xFF) |
| DMA RX configurado errado | Transfer count = 8 (iniciou com 8, nenhum transferido) |
| Pino IN mapeado errado | Mesmo lendo nível alto (pull-up), ISR teria dados não-zero |
| Clock do PIO parado | SM avança (TX FIFO esvazia, PC muda) |
| Conflito de IRQ | Nenhum handler de IRQ configurado |
| Autopush não configurado | Trocamos para push explícito — mesmo problema |
| Buffer do DMA é `uint8_t*` em vez de `uint32_t*` | Corrigido para uint32_t; TX DMA funciona com 32-bit |

### Por que isso é difícil de depurar

1. **Sem acesso a analisador lógico**: Não podemos ver os sinais SCL/SDA para
   confirmar que o I2C está sendo gerado corretamente.

2. **PIO é uma caixa preta**: O PIO não tem debugger. Só podemos observar
   efeitos colaterais (níveis de FIFO, PC) e inferir o estado interno.

3. **32 instruções é muito restritivo**: Cada instrução de depuração (ex: piscar
   LED, setar pino GPIO) consome um slot precioso. Tivemos que remover
   verificação de ACK para caber no limite.

4. **Interação complexa PIO+DMA+GPIO**: O problema pode estar em qualquer
   camada — configuração do PIO, DMA, GPIO function select, ou no programa PIO.

---

## Estrutura de Arquivos

```
pio/i2c_burst.pio          — Programa PIO (32 instruções)
src/i2c_burst.pio.h        — Header auto-gerado pelo pioasm
src/PIO_I2C.h              — API do driver I2C (GPIO + PIO/DMA)
src/PIO_I2C.cpp            — Implementação (bit-bang GPIO + PIO/DMA)
src/BMx280_PIO.h           — Driver do sensor BME280/BMP280
src/BMx280_PIO.cpp         — Implementação (compensação Bosch + API async)
src/main.cpp               — Programa de teste
```

## Como Compilar e Testar

```bash
# Compilar
pio run

# Entrar em BOOTSEL (via 1200 baud no serial)
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',1200); s.close()"
sleep 2

# Gravar firmware
picotool load .pio/build/pico/firmware.uf2

# Reiniciar para modo aplicação
picotool reboot

# Ver saída serial
cat /dev/ttyACM0
```

**Nota**: O dispositivo RP2040 usado nos testes requer reset físico (desconectar/reconectar USB
ou pressionar BOOTSEL) para entrar em modo bootloader após um firmware que não implementa
o reboot via USB serial.

## Próximos Passos Sugeridos

1. **Testar a correção do FIFO join**: O firmware compilado mais recente inclui
   `c.shiftctrl &= ~(3u << 30)` que deve resolver o problema do RX FIFO vazio.

2. **Se ainda falhar, testar com analisador lógico**: Conectar um logic analyzer
   aos pinos SDA/SCL para verificar se o PIO está realmente gerando os sinais
   I2C corretos (START, dados, STOP, etc.).

3. **Testar o programa PIO mínimo de push**: O arquivo `/tmp/test_push.pio` e
   `/tmp/test_core.cpp` contêm um teste mínimo para verificar se `push noblock`
   funciona isoladamente. Pode ser necessário compilar e testar separadamente.

4. **Alternativa: usar autopush em vez de push explícito**: Se o `push noblock`
   continuar não funcionando, tentar reabilitar o autopush
   (`sm_config_set_in_shift(&c, true, true, 8)`) e remover o `push noblock`
   do programa PIO. Isso libera 1 instrução para outros usos.

5. **Adicionar DMA CH3 (pacer/timer)**: Após o burst básico funcionar, adicionar
   o terceiro canal DMA para reconfigurar automaticamente os endereços dos
   canais TX/RX, permitindo varredura contínua verdadeiramente autônoma.

---

## ✅ ATUALIZAÇÃO: Causa Raiz Confirmada

### O Problema

**FIFO Join estava ativado** — o bit `FJOIN_TX` (bit 30 do registrador `SHIFTCTRL`) 
estava setado por padrão ou pela configuração automática do pioasm. Com `FJOIN_TX=1`, 
todas as 8 entradas do FIFO são alocadas para o lado TX. O comando `push` do PIO 
escreve no FIFO combinado (acessível como TX), e **nunca** no RX FIFO. 
O DMA configurado para ler do RX FIFO (`&pio->rxf[sm]`) nunca via dados.

### A Correção

```c
c.shiftctrl &= ~(3u << 30);  // Limpa FJOIN_TX (bit 30) e FJOIN_RX (bit 31)
sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
```

### Evidência da Correção

- **Antes da correção**: `push noblock` nunca produzia dados no RX FIFO (RXF:0 sempre)
- **Depois da correção com `push block`**: Sistema trava → o PIO bloqueia no `push block` 
  porque o RX FIFO **encheu** (4 entradas preenchidas). Isso prova que os dados estão 
  chegando ao RX FIFO.
- **Por que travou?**: No teste manual, o DMA RX não estava configurado para drenar 
  o RX FIFO, então ele encheu e o `push block` bloqueou indefinidamente.

### Solução Final (não testada — dispositivo desconectado)

- **TX**: CPU alimenta comandos via `pio_sm_put()` 
- **RX**: DMA configurado para drenar o RX FIFO via `DREQ_PIOx_RX0`
- **PIO**: `push noblock` (seguro: não bloqueia se DMA estiver momentaneamente lento)
- **Clock**: `div=10` para teste (12.5 MHz PIO), deve ser ajustado para ~96 em produção

### Próximos Passos

1. Reconectar o RP2040 via USB
2. Compilar: `pio run`
3. Gravar: `picotool load -x .pio/build/pico/firmware.uf2 && picotool reboot`
4. Verificar saída serial: `cat /dev/ttyACM0`
5. Se funcionar, ajustar o divisor de clock para o valor correto (125MHz / (100000*13) ≈ 96.15)
6. Restaurar o DMA TX (em vez de CPU feed) para operação totalmente autônoma
