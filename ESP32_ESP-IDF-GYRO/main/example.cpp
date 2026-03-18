/*
 * MPU6050 High-Precision FRC Gyro Implementation
 * 
 * Features:
 *  - DMP (Digital Motion Processing) for 6-axis sensor fusion.
 *  - 10-second OLS linear regression for initial drift estimation.
 *  - ZUPT (Zero-Velocity Update) detector with yaw freeze for zero stationary drift.
 *  - 1D Kalman filter for online gyro bias tracking.
 *  - Runtime temperature compensation for bias stability.
 *  - EMI/I2C glitch rejection for electrically noisy FRC environments.
 */

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"

// ─── Donanım ─────────────────────────────────────────────────────────────────
#define PIN_SDA     21
#define PIN_CLK     22
#define I2C_FREQ_HZ 400000

// ─── Sensör hassasiyeti ───────────────────────────────────────────────────────
#define GYRO_LSB_PER_DPS   131.0f    // FS_SEL=0 → ±250 deg/s
#define ACCEL_LSB_PER_G  16384.0f   // AFS_SEL=0 → ±2 g

// ─── FIFO ────────────────────────────────────────────────────────────────────
#define FIFO_MAX_PACKETS   10

// ─── OLS drift ölçümü ────────────────────────────────────────────────────────
#define OLS_SECONDS   10
#define OLS_HZ       200
#define OLS_SAMPLES  (OLS_SECONDS * OLS_HZ)

// ─── EMI / Glitch rejection ──────────────────────────────────────────────────
// FRC robot max dönüş hızı ~720°/s → 5ms'de max 3.6°
// 5° güvenlik marjıyla (gerçek robotik hareketi kaçırma)
#define MAX_YAW_DELTA_DEG  5.0f

// ─── Gyro Z moving average ───────────────────────────────────────────────────
// ZUPT detection ve Kalman için gürültüyü azaltır.
// 4 tap → varyans /4 → etkili R = R_raw/4
#define GZ_AVG_LEN  4

// ─── Kalman Q / R ─────────────────────────────────────────────────────────────
// Q: bias random-walk varyansı. MPU6050 in-run: ~0.00278 deg/s → (0.00278)² ≈ 7.7e-6
#define KALMAN_Q       7.7e-6f
// R_raw: tek sample gyro Z gürültüsü. noise 0.005 deg/s/√Hz @ 200Hz → σ=0.07 → R=0.005
// R_filt = R_raw / GZ_AVG_LEN (4-tap average)
#define KALMAN_R_FILT  (0.005f / (float)GZ_AVG_LEN)

// ─── ZUPT ────────────────────────────────────────────────────────────────────
#define ZUPT_WINDOW   40            // 200 ms ring buffer @ 200 Hz
#define ZUPT_CONFIRM  20            // 100 ms onay süresi @ 200 Hz  (hızlı tepki)
#define ZUPT_ACCEL_VAR  0.003f      // g² — FRC motor titreşimini tolere eder
#define ZUPT_GYRO_THR   1.0f        // deg/s — filtered

// ─── Sıcaklık ────────────────────────────────────────────────────────────────
#define TEMP_READ_EVERY  20         // sample (100 ms @ 200 Hz)
#define TEMP_MIN_DT       0.3f      // °C — anlamlı ΔT eşiği

// ═════════════════════════════════════════════════════════════════════════════
// Veri yapıları
// ═════════════════════════════════════════════════════════════════════════════

// ─── 1D Kalman ────────────────────────────────────────────────────────────────
typedef struct { float x, P; } KF1D_t;
static inline void kf_predict(KF1D_t *k) { k->P += KALMAN_Q; }
static inline void kf_update(KF1D_t *k, float z, float R) {
    float y = z - k->x;
    float K = k->P / (k->P + R);
    k->x += K * y;
    k->P  = (1.0f - K) * k->P;
}

// ─── ZUPT dedektörü ───────────────────────────────────────────────────────────
typedef struct {
    float buf[ZUPT_WINDOW];
    int   head, n, confirm;
    float sum, sum_sq;
    bool  active;
} ZUPT_t;

static void zupt_reset(ZUPT_t *z) { memset(z, 0, sizeof(*z)); }

static void zupt_push(ZUPT_t *z, float mag) {
    if (z->n == ZUPT_WINDOW) {
        float o = z->buf[z->head];
        z->sum -= o; z->sum_sq -= o * o;
    } else { z->n++; }
    z->buf[z->head] = mag;
    z->sum += mag; z->sum_sq += mag * mag;
    z->head = (z->head + 1) % ZUPT_WINDOW;
}

static bool zupt_update(ZUPT_t *z, float amag, float gz_filt) {
    zupt_push(z, amag);
    bool full = (z->n == ZUPT_WINDOW);
    float mean = z->sum / (float)z->n;
    float var  = full ? (z->sum_sq / (float)z->n - mean * mean) : 9999.0f;
    bool still = full && (var < ZUPT_ACCEL_VAR) && (fabsf(gz_filt) < ZUPT_GYRO_THR);
    if (still) { if (z->confirm < ZUPT_CONFIRM) z->confirm++; }
    else       { z->confirm = 0; z->active = false; }
    if (z->confirm >= ZUPT_CONFIRM) z->active = true;
    return z->active;
}

// ─── Gyro Z 4-tap moving average ──────────────────────────────────────────────
typedef struct {
    float buf[GZ_AVG_LEN];
    int   head;
    float sum;
} GzAvg_t;

static void gzavg_reset(GzAvg_t *g) { memset(g, 0, sizeof(*g)); }
static float gzavg_push(GzAvg_t *g, float val) {
    g->sum -= g->buf[g->head];
    g->buf[g->head] = val;
    g->sum += val;
    g->head = (g->head + 1) % GZ_AVG_LEN;
    return g->sum / (float)GZ_AVG_LEN;
}

// ─── Online sıcaklık modeli ───────────────────────────────────────────────────
typedef struct {
    float  T_ref, T_last, b_last;
    bool   has_last;
    double S_dT, S_db, S_dTdT, S_dTdb;
    int    N;
    float  k;
    bool   k_valid;
} TempModel_t;

static void temp_init(TempModel_t *m, float T0) {
    memset(m, 0, sizeof(*m));
    m->T_ref = T0; m->T_last = T0;
}
static void temp_add(TempModel_t *m, float T, float b) {
    if (m->has_last && fabsf(T - m->T_last) > TEMP_MIN_DT) {
        double dT = T - m->T_ref, db = b;
        m->S_dT += dT; m->S_db += db;
        m->S_dTdT += dT*dT; m->S_dTdb += dT*db;
        m->N++;
        double den = (double)m->N * m->S_dTdT - m->S_dT * m->S_dT;
        if (m->N >= 3 && den > 1e-8) {
            m->k = (float)(((double)m->N * m->S_dTdb - m->S_dT * m->S_db) / den);
            m->k_valid = true;
        }
    }
    m->T_last = T; m->b_last = b; m->has_last = true;
}
static inline float temp_delta(const TempModel_t *m, float T_now) {
    if (!m->k_valid || !m->has_last) return 0.0f;
    float dT = T_now - m->T_last;
    return (fabsf(dT) > 0.05f) ? m->k * dT : 0.0f;
}

// ═════════════════════════════════════════════════════════════════════════════
// Paylaşılan durum
// ═════════════════════════════════════════════════════════════════════════════
static volatile float    g_yawDeg    = 0.0f;
static volatile float    g_yawOffset = 0.0f;
static SemaphoreHandle_t g_yawMutex  = NULL;

static const char *TAG_I2C  = "I2C";
static const char *TAG_MPU  = "MPU";
static const char *TAG_MAIN = "MAIN";

static inline float norm180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// ─── Public API ───────────────────────────────────────────────────────────────
float gyro_getYawDeg(void) {
    float v = g_yawDeg;
    if (xSemaphoreTake(g_yawMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = g_yawDeg; xSemaphoreGive(g_yawMutex);
    }
    return v;
}
void gyro_resetYaw(void) {
    if (xSemaphoreTake(g_yawMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_yawOffset += g_yawDeg; g_yawDeg = 0.0f;
        xSemaphoreGive(g_yawMutex);
        ESP_LOGI(TAG_MAIN, "Yaw sıfırlandı (offset=%.2f°)", g_yawOffset);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// I2C başlatma
// ═════════════════════════════════════════════════════════════════════════════
void task_initI2C(void *ignore) {
    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = (gpio_num_t)PIN_SDA;
    conf.scl_io_num       = (gpio_num_t)PIN_CLK;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    g_yawMutex = xSemaphoreCreateMutex();
    configASSERT(g_yawMutex);
    ESP_LOGI(TAG_I2C, "I2C başlatıldı (%d kHz)", I2C_FREQ_HZ / 1000);
    vTaskDelete(NULL);
}

// ═════════════════════════════════════════════════════════════════════════════
// DMP yardımcıları
// ═════════════════════════════════════════════════════════════════════════════
static Quaternion  q;
static VectorFloat gravity;
static float       ypr[3];
static uint16_t    packetSize = 42;
static uint8_t     fifoBuffer[64];

static bool readDMP(MPU6050 &mpu) {
    uint8_t  st = mpu.getIntStatus();
    uint16_t fc = mpu.getFIFOCount();
    if ((st & 0x10) || fc >= 1024) { mpu.resetFIFO(); return false; }
    if (!(st & 0x02) || fc < packetSize) return false;
    if ((fc / packetSize) > FIFO_MAX_PACKETS) { mpu.resetFIFO(); return false; }
    while (fc >= packetSize) { mpu.getFIFOBytes(fifoBuffer, packetSize); fc -= packetSize; }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return true;
}

static inline float readTempC(MPU6050 &mpu) {
    return mpu.getTemperature() / 340.0f + 36.53f;
}

// ═════════════════════════════════════════════════════════════════════════════
// MPU6050 başlatma
// ═════════════════════════════════════════════════════════════════════════════
static bool initMPU(MPU6050 &mpu) {
    mpu.initialize();
    if (!mpu.testConnection()) { ESP_LOGE(TAG_MPU, "Bağlantı yok!"); return false; }
    if (mpu.dmpInitialize() != 0) { ESP_LOGE(TAG_MPU, "DMP başlatılamadı!"); return false; }
    ESP_LOGI(TAG_MPU, "Isınma (3 sn) – sabit tutun...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG_MPU, "Kalibrasyon başlıyor (CalibrateGyro 15-iter)...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(15);
    ESP_LOGI(TAG_MPU, "Kalibrasyon tamamlandı.");
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    ESP_LOGI(TAG_MPU, "DMP aktif. Paket: %d byte", packetSize);
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// OLS drift ölçümü (10 sn, BLUE estimator)
// ═════════════════════════════════════════════════════════════════════════════
static float olsDriftRate(MPU6050 &mpu, float *T_ref_out) {
    ESP_LOGI(TAG_MPU, "OLS (%d sn): sabit tutun...", OLS_SECONDS);
    double S_x=0,S_y=0,S_xx=0,S_xy=0; double T_sum=0;
    int N=0, T_cnt=0;
    float prev=0, unwrap=0; bool first=true;
    while (N < OLS_SAMPLES) {
        vTaskDelay(pdMS_TO_TICKS(1000/OLS_HZ));
        if (!readDMP(mpu)) continue;
        float raw = ypr[0] * (180.0f / (float)M_PI);
        if (first) { unwrap=raw; prev=raw; first=false; }
        else {
            float d = raw-prev;
            if (d >  180.0f) { d -= 360.0f; }
            if (d < -180.0f) { d += 360.0f; }
            unwrap+=d; prev=raw;
        }
        double t=(double)N/OLS_HZ;
        S_x+=t; S_y+=unwrap; S_xx+=t*t; S_xy+=t*unwrap; N++;
        if ((N%TEMP_READ_EVERY)==0) { T_sum+=readTempC(mpu); T_cnt++; }
    }
    double den=(double)N*S_xx-S_x*S_x;
    float drift=(fabsf((float)den)>1e-12f)
               ?(float)(((double)N*S_xy-S_x*S_y)/den):0.0f;
    *T_ref_out=(T_cnt>0)?(float)(T_sum/T_cnt):25.0f;
    ESP_LOGI(TAG_MPU,
        "OLS bias=%.5f°/s (%.3f°/dak  %.2f°/150sn)  T_ref=%.1f°C",
        drift, drift*60.f, drift*150.f, *T_ref_out);
    return drift;
}

// ═════════════════════════════════════════════════════════════════════════════
// Ana sensör görevi
// ═════════════════════════════════════════════════════════════════════════════
void task_display(void *ignore) {
    MPU6050 mpu;
    if (!initMPU(mpu)) { vTaskDelete(NULL); return; }

    float T_ref    = 25.0f;
    float ols_bias = olsDriftRate(mpu, &T_ref);

    // Kalman: yüksek P → ilk ZUPT'ta hızlı adaptasyon
    KF1D_t      kf    = { ols_bias, 10.0f };
    ZUPT_t      zupt; zupt_reset(&zupt);
    GzAvg_t     gzavg; gzavg_reset(&gzavg);
    TempModel_t tcomp; temp_init(&tcomp, T_ref);

    // Yaw unwrap
    float prev_raw = 0.0f, unwrapped = 0.0f;
    bool  got_first    = false;

    // Düzeltme akümülatörü (ZUPT aktifken dondurulur)
    float accum_corr   = 0.0f;
    const float dt     = 1.0f / 200.0f;   // 5 ms

    // ZUPT freeze state
    bool  zupt_prev    = false;
    float yaw_frozen   = 0.0f;

    // Sıcaklık cache
    float T_now = T_ref; int temp_tmr = 0;

    // Glitch sayacı (tanılama için)
    uint32_t glitch_count = 0;

    ESP_LOGI(TAG_MAIN, "Başladı. OLS_bias=%.5f°/s  T=%.1f°C", ols_bias, T_ref);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5));    // 200 Hz
        if (!readDMP(mpu)) continue;

        // ── [3] EMI Glitch Rejection ────────────────────────────────────────
        float raw = ypr[0] * (180.0f / (float)M_PI);
        if (!got_first) {
            unwrapped = raw; prev_raw = raw; got_first = true;
        } else {
            float delta = raw - prev_raw;
            if (delta >  180.0f) delta -= 360.0f;
            if (delta < -180.0f) delta += 360.0f;

            if (fabsf(delta) <= MAX_YAW_DELTA_DEG) {
                // Geçerli delta: integrate et
                unwrapped += delta;
                prev_raw   = raw;
            } else {
                // EMI/glitch: bu sample'ı reddet, prev_raw değişmesin
                glitch_count++;
                if ((glitch_count % 10) == 1) {   // log'u boğma
                    ESP_LOGW(TAG_MAIN, "EMI glitch #%lu: delta=%.1f°", glitch_count, delta);
                }
            }
        }

        // ── Sıcaklık (100 ms'de bir) ─────────────────────────────────────────
        if (++temp_tmr >= TEMP_READ_EVERY) { T_now = readTempC(mpu); temp_tmr = 0; }

        // ── Ham sensörler ────────────────────────────────────────────────────
        int16_t gx_r, gy_r, gz_r;
        mpu.getRotation(&gx_r, &gy_r, &gz_r);
        float gz_raw = gz_r / GYRO_LSB_PER_DPS;

        // ── [4] Gz moving average ────────────────────────────────────────────
        float gz_filt = gzavg_push(&gzavg, gz_raw);

        int16_t ax_r, ay_r, az_r;
        mpu.getAcceleration(&ax_r, &ay_r, &az_r);
        float ax = ax_r / ACCEL_LSB_PER_G;
        float ay = ay_r / ACCEL_LSB_PER_G;
        float az = az_r / ACCEL_LSB_PER_G;
        float amag = sqrtf(ax*ax + ay*ay + az*az);

        // ── [5] Kalman predict ───────────────────────────────────────────────
        kf_predict(&kf);

        // ── [6] ZUPT detection (filtered gz + accel variance) ───────────────
        bool is_still = zupt_update(&zupt, amag, gz_filt);

        // ── [7] ZUPT Yaw Freeze State Machine ──────────────────────────────
        if (is_still) {
            // Rising edge: yaw'ı dondur
            if (!zupt_prev) {
                yaw_frozen = norm180(unwrapped - accum_corr - g_yawOffset);
                ESP_LOGI(TAG_MAIN, "ZUPT ON  frozen=%.3f°", yaw_frozen);
            }
            // [5] Kalman update: gz_filt tamamen bias (filtered ölçüm, R_filt kullan)
            kf_update(&kf, gz_filt, KALMAN_R_FILT);
            // [8] Sıcaklık modeli güncelle
            temp_add(&tcomp, T_now, kf.x);
            // accum_corr GÜNCELLEME (çıkış donduruldu)
            // Çıkış: frozen yaw
            if (xSemaphoreTake(g_yawMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                g_yawDeg = yaw_frozen; xSemaphoreGive(g_yawMutex);
            }
        } else {
            // Falling edge: accum_corr'u frozen yaw ile senkronize et
            if (zupt_prev) {
                accum_corr = unwrapped - g_yawOffset - yaw_frozen;
                ESP_LOGI(TAG_MAIN, "ZUPT OFF  bias=%.5f°/s  kT=%.5f°/s/°C  glitch=%lu",
                         kf.x, tcomp.k, glitch_count);
            }
            // [8] Sıcaklık düzeltmesi + bias
            float eff = kf.x + temp_delta(&tcomp, T_now);
            accum_corr += eff * dt;
            float yaw_out = norm180(unwrapped - accum_corr - g_yawOffset);
            if (xSemaphoreTake(g_yawMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                g_yawDeg = yaw_out; xSemaphoreGive(g_yawMutex);
            }
        }
        zupt_prev = is_still;

        // ── Serial çıktı ─────────────────────────────────────────────────────
        printf("YAW:%7.2f°  P:%6.2f°  R:%6.2f°"
               "  bias:%+.5f  kT:%+.5f  T:%5.1f°C  [%s]  glitch:%lu\n",
               g_yawDeg,
               ypr[1] * (180.0f / (float)M_PI),
               ypr[2] * (180.0f / (float)M_PI),
               kf.x, tcomp.k, T_now,
               is_still ? "ZUPT" : "    ",
               glitch_count);
    }
    vTaskDelete(NULL);
}
