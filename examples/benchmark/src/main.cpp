/**
 * @example BMx280PIO_RP2040 vs Adafruit BMP280 — Dual-Core Stress Benchmark
 * @author Ângelo Moisés Alves (@angeloINTJ)
 *
 * Compara 3 modos de transporte I2C com ambos os núcleos sob carga matemática:
 *   1. PIO+DMA  (burstRead) — WirePIO @ 200 kHz
 *   2. GPIO bit-bang          — forced fallback @ 100 kHz
 *   3. Adafruit BMP280        — hardware Wire I2C @ 100 kHz
 *
 * Hardware: Raspberry Pi Pico com 2x BMP280
 *   Sensor A: GPIO4 (SDA), GPIO5 (SCL) — endereço 0x76
 *   Sensor B: GPIO6 (SDA), GPIO7 (SCL) — endereço 0x76
 *
 * 1000 leituras por sensor por modo = 6000 leituras totais.
 * Ambos os núcleos executam tarefas matemáticas pesadas durante o teste.
 */

#include <Arduino.h>
#include "BMx280PIO_RP2040.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <stdlib.h>

// ─── Constantes ─────────────────────────────────────────────────────────
#define SENSOR_A_SDA  4
#define SENSOR_A_SCL  5
#define SENSOR_B_SDA  6
#define SENSOR_B_SCL  7
#define BMP280_ADDR   0x76
#define NUM_READINGS  1000
#define WARMUP        50

// ─── Stress Matemático (Core 1) ────────────────────────────────────────
volatile bool     g_stressActive = false;
volatile uint32_t g_stressOps    = 0;

// Core 1: loop contínuo com Mandelbrot + operações trigonométricas
// NOTA: freertos-main.cpp declara setup1/loop1 com C++ linkage (sem extern "C")

void setup1() {
    // Core 1 não usa Serial — apenas matemática
}

void loop1() {
    if (!g_stressActive) {
        delay(1);
        return;
    }

    // Mandelbrot set iteration — FPU-intensive com branches imprevisíveis
    float cx = -0.7269f, cy = 0.1889f;
    float x = 0.0f, y = 0.0f;
    int iter = 0;
    while (x*x + y*y <= 4.0f && iter < 100) {
        float xtemp = x*x - y*y + cx;
        y = 2.0f * x * y + cy;
        x = xtemp;
        iter++;
    }

    // Mix de operações transcendentes para saturar a FPU
    volatile float v = (float)iter;
    v = sinf(v) * cosf(v) + sqrtf(fabsf(v));
    v = atanf(v) * expf(fminf(fabsf(v), 10.0f));
    v = logf(fabsf(v) + 1.0f) * tanf(v * 0.1f);

    g_stressOps++;
}

// Stress no Core 0 entre leituras
static void core0Stress(int n) {
    volatile float x = 2.718281828f;
    for (int i = 0; i < n; i++) {
        x = sinf(x) * cosf(x) + sqrtf(fabsf(x));
        x = x * 1.0001f + 0.0001f;
        x = atanf(x) * expf(fminf(fabsf(x), 10.0f));
        x = logf(fabsf(x) + 1.0f) * tanf(x * 0.1f);
    }
}

// ─── Estatísticas ───────────────────────────────────────────────────────
struct ReadingStats {
    uint32_t count, errors;
    float    t_min, t_max;
    double   t_sum;
    float    p_min, p_max;
    double   p_sum;
    uint32_t time_min, time_max;
    uint64_t time_sum;          // soma em µs (cabe em 64 bits)
    uint32_t times[NUM_READINGS];
};

static void initStats(ReadingStats &s) {
    s.count = 0; s.errors = 0;
    s.t_min = 999.0f; s.t_max = -999.0f;
    s.t_sum = 0.0;
    s.p_min = 999999.0f; s.p_max = -999999.0f;
    s.p_sum = 0.0;
    s.time_min = 0xFFFFFFFF; s.time_max = 0;
    s.time_sum = 0;
    memset(s.times, 0, sizeof(s.times));
}

static void recordReading(ReadingStats &s, float temp, float pres, uint32_t dt_us) {
    if (s.count >= NUM_READINGS) return;
    s.times[s.count] = dt_us;
    s.count++;

    if (!isnan(temp) && !isnan(pres)) {
        if (temp < s.t_min) s.t_min = temp;
        if (temp > s.t_max) s.t_max = temp;
        s.t_sum += temp;
        if (pres < s.p_min) s.p_min = pres;
        if (pres > s.p_max) s.p_max = pres;
        s.p_sum += pres;
    } else {
        s.errors++;
    }

    if (dt_us < s.time_min) s.time_min = dt_us;
    if (dt_us > s.time_max) s.time_max = dt_us;
    s.time_sum += dt_us;
}

static int cmp_u32(const void *a, const void *b) {
    uint32_t va = *(const uint32_t*)a;
    uint32_t vb = *(const uint32_t*)b;
    return (va > vb) - (va < vb);
}

// Estrutura de resumo por fase
struct PhaseSummary {
    const char *name;
    // Sensor A
    uint32_t a_count, a_errors, a_time_min, a_time_max, a_time_med, a_time_p99;
    float    a_time_mean, a_time_sd, a_reads_per_sec;
    float    a_tmin, a_tmax, a_tmean;
    float    a_pmin, a_pmax, a_pmean;
    // Sensor B
    uint32_t b_count, b_errors, b_time_min, b_time_max, b_time_med, b_time_p99;
    float    b_time_mean, b_time_sd, b_reads_per_sec;
    float    b_tmin, b_tmax, b_tmean;
    float    b_pmin, b_pmax, b_pmean;
    // Meta
    uint32_t stress_ops, duration_ms;
};

PhaseSummary g_summaries[3];

static void finalizeStats(ReadingStats &s, PhaseSummary &sum, const char *name) {
    sum.name = name;

    // Ordena tempos para mediana e percentis
    qsort(s.times, s.count, sizeof(uint32_t), cmp_u32);

    sum.a_count  = s.count;
    sum.a_errors = s.errors;
    sum.a_tmin   = s.t_min;
    sum.a_tmax   = s.t_max;
    sum.a_tmean  = s.count > s.errors ? (float)(s.t_sum / (s.count - s.errors)) : NAN;
    sum.a_pmin   = s.p_min;
    sum.a_pmax   = s.p_max;
    sum.a_pmean  = s.count > s.errors ? (float)(s.p_sum / (s.count - s.errors)) : NAN;

    sum.a_time_min = s.time_min;
    sum.a_time_max = s.time_max;
    sum.a_time_med = s.times[s.count / 2];
    sum.a_time_p99 = s.times[(uint32_t)(s.count * 0.99)];
    sum.a_time_mean = s.count > 0 ? (float)((double)s.time_sum / s.count) : 0.0f;

    // Desvio padrão (two-pass)
    double sd = 0.0;
    for (uint32_t i = 0; i < s.count; i++) {
        double d = (double)s.times[i] - sum.a_time_mean;
        sd += d * d;
    }
    sum.a_time_sd = s.count > 0 ? (float)sqrt(sd / s.count) : 0.0f;

    // Leituras por segundo (usa o tempo total real: time_sum cobre todas as leituras)
    sum.a_reads_per_sec = s.time_sum > 0 ? (float)(s.count * 1000000ULL) / (float)s.time_sum : 0.0f;
}

// Copia de A para B (estrutura simétrica — benchmark usa 2 sensores idênticos)
static void copySummaryB(PhaseSummary &sum, ReadingStats &s) {
    qsort(s.times, s.count, sizeof(uint32_t), cmp_u32);
    sum.b_count  = s.count;
    sum.b_errors = s.errors;
    sum.b_tmin   = s.t_min;
    sum.b_tmax   = s.t_max;
    sum.b_tmean  = s.count > s.errors ? (float)(s.t_sum / (s.count - s.errors)) : NAN;
    sum.b_pmin   = s.p_min;
    sum.b_pmax   = s.p_max;
    sum.b_pmean  = s.count > s.errors ? (float)(s.p_sum / (s.count - s.errors)) : NAN;
    sum.b_time_min = s.time_min;
    sum.b_time_max = s.time_max;
    sum.b_time_med = s.times[s.count / 2];
    sum.b_time_p99 = s.times[(uint32_t)(s.count * 0.99)];
    sum.b_time_mean = s.count > 0 ? (float)((double)s.time_sum / s.count) : 0.0f;
    double sd = 0.0;
    for (uint32_t i = 0; i < s.count; i++) {
        double d = (double)s.times[i] - sum.b_time_mean;
        sd += d * d;
    }
    sum.b_time_sd = s.count > 0 ? (float)sqrt(sd / s.count) : 0.0f;
    sum.b_reads_per_sec = s.time_sum > 0 ? (float)(s.count * 1000000ULL) / (float)s.time_sum : 0.0f;
}

// ─── Funções de Benchmark ───────────────────────────────────────────────

// Bloco comum: aquece, ativa stress, mede 1000 leituras intercaladas
template<typename ReadFnA, typename ReadFnB>
void runBenchPhase(const char *label, int phaseIdx,
                   ReadFnA readA, ReadFnB readB) {
    ReadingStats sA, sB;
    initStats(sA);
    initStats(sB);

    Serial.print("\n\n=== ");
    Serial.print(label);
    Serial.println(" ===\n");

    // Warmup (leituras descartadas)
    g_stressActive = true;
    delay(50);
    for (int i = 0; i < WARMUP; i++) {
        core0Stress(20);
        float ta, pa, tb, pb;
        readA(&ta, &pa);
        readB(&tb, &pb);
        delayMicroseconds(500);
    }

    // Reseta contador de stress
    g_stressOps = 0;
    uint32_t tStart = millis();

    // ─── 1000 leituras ──────────────────────────────────────────────
    for (int i = 0; i < NUM_READINGS; i++) {
        // Stress no core 0 entre leituras
        core0Stress(30);

        // Sensor A
        float ta = NAN, pa = NAN;
        uint32_t t0 = micros();
        readA(&ta, &pa);
        uint32_t dtA = micros() - t0;
        recordReading(sA, ta, pa, dtA);

        // Pequena pausa para não saturar o barramento
        delayMicroseconds(200);
        core0Stress(15);

        // Sensor B
        float tb = NAN, pb = NAN;
        t0 = micros();
        readB(&tb, &pb);
        uint32_t dtB = micros() - t0;
        recordReading(sB, tb, pb, dtB);

        delayMicroseconds(200);

        // Progresso
        if ((i + 1) % 100 == 0) {
            Serial.print("  Leitura ");
            Serial.print(i + 1);
            Serial.print("/");
            Serial.print(NUM_READINGS);
            Serial.print(" | A: ");
            Serial.print(ta, 2);
            Serial.print(" C, ");
            Serial.print(pa, 2);
            Serial.print(" hPa | B: ");
            Serial.print(tb, 2);
            Serial.print(" C, ");
            Serial.print(pb, 2);
            Serial.print(" hPa | dtA=");
            Serial.print(dtA);
            Serial.print("us dtB=");
            Serial.print(dtB);
            Serial.println("us");
        }
    }

    uint32_t tEnd = millis();
    g_stressActive = false;

    // Finaliza estatísticas
    PhaseSummary &sum = g_summaries[phaseIdx];
    sum.duration_ms = tEnd - tStart;
    sum.stress_ops  = g_stressOps;

    finalizeStats(sA, sum, label);
    sum.a_count  = sA.count;   // restore after finalizeStats overwrites name
    sum.a_errors = sA.errors;
    sum.a_tmin   = sA.t_min;
    sum.a_tmax   = sA.t_max;
    sum.a_tmean  = sA.count > sA.errors ? (float)(sA.t_sum / (sA.count - sA.errors)) : NAN;
    sum.a_pmin   = sA.p_min;
    sum.a_pmax   = sA.p_max;
    sum.a_pmean  = sA.count > sA.errors ? (float)(sA.p_sum / (sA.count - sA.errors)) : NAN;
    sum.a_time_min = sA.time_min;
    sum.a_time_max = sA.time_max;
    sum.a_time_med = sA.times[sA.count / 2];
    sum.a_time_p99 = sA.times[(uint32_t)(sA.count * 0.99)];
    sum.a_time_mean = sA.count > 0 ? (float)((double)sA.time_sum / sA.count) : 0.0f;
    {
        double sd = 0.0;
        for (uint32_t i = 0; i < sA.count; i++) {
            double d = (double)sA.times[i] - sum.a_time_mean;
            sd += d * d;
        }
        sum.a_time_sd = sA.count > 0 ? (float)sqrt(sd / sA.count) : 0.0f;
        sum.a_reads_per_sec = sA.time_sum > 0 ? (float)(sA.count * 1000000ULL) / (float)sA.time_sum : 0.0f;
    }
    copySummaryB(sum, sB);

    // ─── Relatório da Fase ──────────────────────────────────────────
    Serial.println();
    Serial.println("──────────────────────────────────────────────────────────");
    Serial.print("  RESULTADO: ");
    Serial.println(label);
    Serial.println("──────────────────────────────────────────────────────────");
    Serial.printf("  Duração:          %lu ms\n", sum.duration_ms);
    Serial.printf("  Stress Core 1:    %lu operações\n", sum.stress_ops);
    Serial.println();

    auto printSensorStats = [](const char *tag, uint32_t cnt, uint32_t err,
                                float tmean, float tmin, float tmax,
                                float pmean, float pmin, float pmax,
                                uint32_t tmin_us, uint32_t tmax_us,
                                uint32_t tmed, uint32_t tp99,
                                float tmean_us, float tsd, float rps) {
        Serial.printf("  ── Sensor %s ─────────────────────────────────\n", tag);
        Serial.printf("  Leituras:          %lu  (erros: %lu)\n", cnt, err);
        Serial.printf("  Temperatura:       %.2f C  [%.2f .. %.2f]\n", tmean, tmin, tmax);
        Serial.printf("  Pressão:           %.2f hPa [%.2f .. %.2f]\n", pmean, pmin, pmax);
        Serial.println();
        Serial.printf("  Tempo/leitura min: %lu µs\n", tmin_us);
        Serial.printf("  Tempo/leitura max: %lu µs\n", tmax_us);
        Serial.printf("  Tempo/leitura med: %lu µs\n", tmed);
        Serial.printf("  Tempo/leitura P99: %lu µs\n", tp99);
        Serial.printf("  Tempo/leitura méd: %.1f µs\n", tmean_us);
        Serial.printf("  Desvio padrão:     %.1f µs\n", tsd);
        Serial.printf("  Throughput:        %.1f leituras/s\n", rps);
        Serial.println();
    };

    printSensorStats("A (4/5)", sum.a_count, sum.a_errors,
                     sum.a_tmean, sum.a_tmin, sum.a_tmax,
                     sum.a_pmean, sum.a_pmin, sum.a_pmax,
                     sum.a_time_min, sum.a_time_max,
                     sum.a_time_med, sum.a_time_p99,
                     sum.a_time_mean, sum.a_time_sd,
                     sum.a_reads_per_sec);

    printSensorStats("B (6/7)", sum.b_count, sum.b_errors,
                     sum.b_tmean, sum.b_tmin, sum.b_tmax,
                     sum.b_pmean, sum.b_pmin, sum.b_pmax,
                     sum.b_time_min, sum.b_time_max,
                     sum.b_time_med, sum.b_time_p99,
                     sum.b_time_mean, sum.b_time_sd,
                     sum.b_reads_per_sec);
}

// ─── Fase 1: PIO+DMA ───────────────────────────────────────────────────
void runPhase_PIO() {
    BMx280PIO_RP2040 sensorA(SENSOR_A_SDA, SENSOR_A_SCL, BMP280_ADDR, 200000, pio0);
    BMx280PIO_RP2040 sensorB(SENSOR_B_SDA, SENSOR_B_SCL, BMP280_ADDR, 200000, pio0);

    Serial.print("Inicializando Sensor A (PIO pio0)... ");
    if (!sensorA.begin()) {
        Serial.println("FALHA!"); return;
    }
    Serial.println(sensorA.isBME280() ? "BME280 OK" : "BMP280 OK");
    sensorA.setMode(BME280_MODE_NORMAL);
    delay(50);

    Serial.print("Inicializando Sensor B (PIO pio0)... ");
    if (!sensorB.begin()) {
        Serial.println("FALHA!"); return;
    }
    Serial.println(sensorB.isBME280() ? "BME280 OK" : "BMP280 OK");
    sensorB.setMode(BME280_MODE_NORMAL);
    delay(50);

    auto readA = [&sensorA](float *t, float *p) {
        sensorA.readAll(t, p, nullptr);
    };
    auto readB = [&sensorB](float *t, float *p) {
        sensorB.readAll(t, p, nullptr);
    };

    runBenchPhase("MODO 1: PIO+DMA (WirePIO burstRead @ 200 kHz)", 0, readA, readB);
}

// ─── Fase 2: GPIO bit-bang ─────────────────────────────────────────────
void runPhase_GPIO() {
    // Mesmo construtor, mas forceGPIO(true) evita PIO — usa bit-bang puro
    BMx280PIO_RP2040 sensorA(SENSOR_A_SDA, SENSOR_A_SCL, BMP280_ADDR);
    BMx280PIO_RP2040 sensorB(SENSOR_B_SDA, SENSOR_B_SCL, BMP280_ADDR);
    sensorA.forceGPIO(true);
    sensorB.forceGPIO(true);

    Serial.print("Inicializando Sensor A (GPIO bit-bang)... ");
    if (!sensorA.begin()) {
        Serial.println("FALHA!"); return;
    }
    Serial.println(sensorA.isBME280() ? "BME280 OK" : "BMP280 OK");
    sensorA.setMode(BME280_MODE_NORMAL);
    delay(50);

    Serial.print("Inicializando Sensor B (GPIO bit-bang)... ");
    if (!sensorB.begin()) {
        Serial.println("FALHA!"); return;
    }
    Serial.println(sensorB.isBME280() ? "BME280 OK" : "BMP280 OK");
    sensorB.setMode(BME280_MODE_NORMAL);
    delay(50);

    auto readA = [&sensorA](float *t, float *p) {
        sensorA.readAll(t, p, nullptr);
    };
    auto readB = [&sensorB](float *t, float *p) {
        sensorB.readAll(t, p, nullptr);
    };

    runBenchPhase("MODO 2: GPIO bit-bang (forced fallback @ 100 kHz)", 1, readA, readB);
}

// ─── Fase 3: Adafruit BMP280 (hardware Wire) ───────────────────────────
void runPhase_Adafruit() {
    // Configura I2C por hardware
    Wire.setSDA(SENSOR_A_SDA);
    Wire.setSCL(SENSOR_A_SCL);
    Wire.begin();
    Wire.setClock(100000);

    Wire1.setSDA(SENSOR_B_SDA);
    Wire1.setSCL(SENSOR_B_SCL);
    Wire1.begin();
    Wire1.setClock(100000);

    Adafruit_BMP280 bmpA(&Wire);
    Adafruit_BMP280 bmpB(&Wire1);

    Serial.print("Inicializando Sensor A (Adafruit Wire I2C0)... ");
    if (!bmpA.begin(BMP280_ADDR)) {
        Serial.println("FALHA!"); return;
    }
    Serial.println("BMP280 OK");
    bmpA.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X1,
                     Adafruit_BMP280::SAMPLING_X1,
                     Adafruit_BMP280::FILTER_OFF,
                     Adafruit_BMP280::STANDBY_MS_1);
    delay(50);

    Serial.print("Inicializando Sensor B (Adafruit Wire I2C1)... ");
    if (!bmpB.begin(BMP280_ADDR)) {
        Serial.println("FALHA!"); return;
    }
    Serial.println("BMP280 OK");
    bmpB.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X1,
                     Adafruit_BMP280::SAMPLING_X1,
                     Adafruit_BMP280::FILTER_OFF,
                     Adafruit_BMP280::STANDBY_MS_1);
    delay(50);

    auto readA = [&bmpA](float *t, float *p) {
        *t = bmpA.readTemperature();
        *p = bmpA.readPressure() / 100.0f;  // Pa → hPa
    };
    auto readB = [&bmpB](float *t, float *p) {
        *t = bmpB.readTemperature();
        *p = bmpB.readPressure() / 100.0f;  // Pa → hPa
    };

    runBenchPhase("MODO 3: Adafruit BMP280 (hardware Wire @ 100 kHz)", 2, readA, readB);

    // Libera hardware I2C
    Wire.end();
    Wire1.end();
}

// ─── Sumário Final Comparativo ──────────────────────────────────────────
static void printFinalSummary() {
    Serial.println("\n\n");
    Serial.println("╔══════════════════════════════════════════════════════════════════╗");
    Serial.println("║              SUMÁRIO COMPARATIVO FINAL                           ║");
    Serial.println("║  2x BMP280 | 1000 leituras/modo | Dual-Core Stress               ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════╣");

    // Cabeçalho
    Serial.println("║                    │   PIO+DMA   │  GPIO bit-bang │  Adafruit    ║");
    Serial.println("╠════════════════════╪═════════════╪════════════════╪══════════════╣");

    auto printRow = [](const char *label, const char *fmt,
                       float v0, float v1, float v2, const char *unit) {
        char buf[128];
        snprintf(buf, sizeof(buf), "║ %-18s │ ", label);
        Serial.print(buf);
        snprintf(buf, sizeof(buf), fmt, v0);
        Serial.print(buf);
        Serial.print("  │ ");
        snprintf(buf, sizeof(buf), fmt, v1);
        Serial.print(buf);
        Serial.print("  │ ");
        snprintf(buf, sizeof(buf), fmt, v2);
        Serial.print(buf);
        Serial.print(" ");
        Serial.print(unit);
        Serial.println(" ║");
    };

    auto printRowU32 = [](const char *label,
                          uint32_t v0, uint32_t v1, uint32_t v2, const char *unit) {
        char buf[128];
        snprintf(buf, sizeof(buf), "║ %-18s │ %8lu %-5s│ %8lu %-5s│ %8lu %-5s║",
                 label, v0, unit, v1, unit, v2, unit);
        Serial.println(buf);
    };

    PhaseSummary &pio = g_summaries[0];
    PhaseSummary &gpio = g_summaries[1];
    PhaseSummary &ada  = g_summaries[2];

    Serial.println("║── Sensor A (GPIO 4/5) ───────────────────────────────────────────╣");

    printRowU32("Leituras OK", pio.a_count - pio.a_errors, gpio.a_count - gpio.a_errors, ada.a_count - ada.a_errors, "");
    printRowU32("Erros", pio.a_errors, gpio.a_errors, ada.a_errors, "");
    printRow("Temperatura média", "%8.2f", pio.a_tmean, gpio.a_tmean, ada.a_tmean, "C  ");
    printRow("Pressão média", "%8.2f", pio.a_pmean, gpio.a_pmean, ada.a_pmean, "hPa");

    Serial.println("║── Tempo por Leitura Sensor A ────────────────────────────────────╣");

    printRowU32("Mínimo (µs)", pio.a_time_min, gpio.a_time_min, ada.a_time_min, "µs");
    printRowU32("Mediana (µs)", pio.a_time_med, gpio.a_time_med, ada.a_time_med, "µs");
    printRow("Média (µs)", "%8.1f", pio.a_time_mean, gpio.a_time_mean, ada.a_time_mean, "µs ");
    printRowU32("P99 (µs)", pio.a_time_p99, gpio.a_time_p99, ada.a_time_p99, "µs");
    printRowU32("Máximo (µs)", pio.a_time_max, gpio.a_time_max, ada.a_time_max, "µs");
    printRow("Desvio Padrão", "%8.1f", pio.a_time_sd, gpio.a_time_sd, ada.a_time_sd, "µs ");
    printRow("Throughput", "%8.1f", pio.a_reads_per_sec, gpio.a_reads_per_sec, ada.a_reads_per_sec, "r/s ");

    Serial.println("║── Sensor B (GPIO 6/7) ───────────────────────────────────────────╣");

    printRowU32("Leituras OK", pio.b_count - pio.b_errors, gpio.b_count - gpio.b_errors, ada.b_count - ada.b_errors, "");
    printRowU32("Erros", pio.b_errors, gpio.b_errors, ada.b_errors, "");
    printRow("Temperatura média", "%8.2f", pio.b_tmean, gpio.b_tmean, ada.b_tmean, "C  ");
    printRow("Pressão média", "%8.2f", pio.b_pmean, gpio.b_pmean, ada.b_pmean, "hPa");

    Serial.println("║── Tempo por Leitura Sensor B ────────────────────────────────────╣");

    printRowU32("Mínimo (µs)", pio.b_time_min, gpio.b_time_min, ada.b_time_min, "µs");
    printRowU32("Mediana (µs)", pio.b_time_med, gpio.b_time_med, ada.b_time_med, "µs");
    printRow("Média (µs)", "%8.1f", pio.b_time_mean, gpio.b_time_mean, ada.b_time_mean, "µs ");
    printRowU32("P99 (µs)", pio.b_time_p99, gpio.b_time_p99, ada.b_time_p99, "µs");
    printRowU32("Máximo (µs)", pio.b_time_max, gpio.b_time_max, ada.b_time_max, "µs");
    printRow("Desvio Padrão", "%8.1f", pio.b_time_sd, gpio.b_time_sd, ada.b_time_sd, "µs ");
    printRow("Throughput", "%8.1f", pio.b_reads_per_sec, gpio.b_reads_per_sec, ada.b_reads_per_sec, "r/s ");

    Serial.println("║── Meta ──────────────────────────────────────────────────────────╣");

    printRowU32("Duração (ms)", pio.duration_ms, gpio.duration_ms, ada.duration_ms, "ms");
    printRowU32("Stress ops", pio.stress_ops, gpio.stress_ops, ada.stress_ops, "");

    // Speedup relativo
    Serial.println("║── Speedup (vs Adafruit) ─────────────────────────────────────────╣");

    float pioSpeedupA = ada.a_time_mean > 0 ? ada.a_time_mean / pio.a_time_mean : 0.0f;
    float gpioSpeedupA = ada.a_time_mean > 0 ? ada.a_time_mean / gpio.a_time_mean : 0.0f;
    float pioSpeedupB = ada.b_time_mean > 0 ? ada.b_time_mean / pio.b_time_mean : 0.0f;
    float gpioSpeedupB = ada.b_time_mean > 0 ? ada.b_time_mean / gpio.b_time_mean : 0.0f;

    printRow("Speedup A (PIO)", "%8.2fx", pioSpeedupA, 0.0f, 0.0f, "");
    printRow("Speedup A (GPIO)", "%8.2fx", gpioSpeedupA, 0.0f, 0.0f, "");
    printRow("Speedup B (PIO)", "%8.2fx", pioSpeedupB, 0.0f, 0.0f, "");
    printRow("Speedup B (GPIO)", "%8.2fx", gpioSpeedupB, 0.0f, 0.0f, "");

    Serial.println("╚══════════════════════════════════════════════════════════════════╝");

    // Legenda
    Serial.println();
    Serial.println("NOTAS:");
    Serial.println("  • PIO+DMA:     WirePIO com burstRead DMA, I2C @ 200 kHz");
    Serial.println("  • GPIO bit-bang: Software bit-bang nos GPIOs, I2C @ 100 kHz");
    Serial.println("  • Adafruit:    Hardware I2C (Wire/Wire1), I2C @ 100 kHz");
    Serial.println("  • Ambos os núcleos sob carga matemática (Mandelbrot + trig)");
    Serial.println("  • Temperatura/Pressão com oversampling 1x, filtro desligado");
    Serial.println();
    Serial.println("=== FIM DO BENCHMARK ===");
}

// ─── Setup / Loop ───────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(2000);

    Serial.println("\n\n");
    Serial.println("╔══════════════════════════════════════════════════════════╗");
    Serial.println("║   BMx280PIO_RP2040 vs Adafruit BMP280                   ║");
    Serial.println("║   Benchmark Dual-Core — 2x BMP280 — 3 modos I2C         ║");
    Serial.println("║   1000 leituras / sensor / modo = 6000 leituras totais  ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Configuração:");
    Serial.println("  Sensor A: GPIO4 (SDA), GPIO5 (SCL) — endereço 0x76");
    Serial.println("  Sensor B: GPIO6 (SDA), GPIO7 (SCL) — endereço 0x76");
    Serial.println("  Oversampling: 1x | Filtro: OFF | Modo: NORMAL");
    Serial.println("  Core 0: leituras + stress matemático");
    Serial.println("  Core 1: Mandelbrot + operações transcendentes");

    // ─── Executa as 3 fases ────────────────────────────────────────
    runPhase_PIO();
    delay(500);

    runPhase_GPIO();
    delay(500);

    runPhase_Adafruit();

    // ─── Sumário Final ─────────────────────────────────────────────
    printFinalSummary();
}

void loop() {
    delay(1000);
}
