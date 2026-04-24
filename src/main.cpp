/*
 * main.cpp – BU04 Ai-Thinker UWB: дистанция + направление
 *
 * Три роли компилируются из одного файла (build_flags):
 *   -DROLE_ANCHOR1=1   главный якорь на платформе (считает направление)
 *   -DROLE_ANCHOR2=1   ведомый якорь на платформе (шлёт d2 по ESP-NOW)
 *   -DROLE_TAG=1       тег в руках пользователя
 *
 * BU04 управляется AT-командами по UART (PA2/PA3 = USART2).
 * Чип DW3000 скрыт за STM32F103, доступен только через AT-интерфейс.
 *
 * ───────────── Геометрия направления (2D) ─────────────────
 *
 *   ANCHOR1 ──── B (baseline) ──── ANCHOR2
 *    (0, 0)                         (B, 0)
 *
 *       TAG(x, y)
 *
 *   Из теоремы косинусов / перпендикуляра:
 *     x   = (d1² - d2² + B²) / (2·B)
 *     y   = √(d1² - x²)          ← y > 0, сторона не определена без 3-го якоря
 *     β   = atan2(y, x)           ← угол от оси ANCHOR1→ANCHOR2
 *
 *   Примечание: DW3000 поддерживает PDOA (фазовую разность приёма),
 *   что теоретически позволяет определить угол одним модулем.
 *   Однако AT-команда для PDOA в текущей прошивке BU04 не задокументирована.
 *   Код готов к добавлению PDOA: см. метку TODO_PDOA.
 *
 * ───────────── Связь между якорями ────────────────────────
 *   ANCHOR2 → ANCHOR1 через ESP-NOW (WiFi-чип ESP32-C3, без проводов)
 *
 * ───────────── Формат телеметрии (CSV, 115200 бод) ─────────
 *   PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms
 *   ANCHOR1,seq,d1_m,avg_m,ts_ms
 *   ANCHOR2,seq,d2_m,ts_ms
 *   TAG,seq,d_m,avg_m,ts_ms
 *   IMU,<сырые данные BU04>,ts_ms
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include "config.h"

#if !defined(ROLE_ANCHOR1) && !defined(ROLE_ANCHOR2) && !defined(ROLE_TAG)
#  error "Задайте ROLE_ANCHOR1=1, ROLE_ANCHOR2=1 или ROLE_TAG=1 в build_flags"
#endif

// ESP-NOW нужен только якорям
#if defined(ROLE_ANCHOR1) || defined(ROLE_ANCHOR2)
#  include <WiFi.h>
#  include <esp_now.h>
#  include <esp_idf_version.h>
#endif

// ── UART к BU04 ──────────────────────────────────────────────
static HardwareSerial bu04(1);   // UART1 ESP32-C3, пины заданы в config.h

// ── Общие переменные ─────────────────────────────────────────
static uint8_t  g_seq     = 0;
static float    g_avgBuf[MOVING_AVG_SAMPLES] = {};
static uint8_t  g_avgIdx  = 0;
static bool     g_avgFull = false;

// ── ANCHOR1: данные от ANCHOR2 ───────────────────────────────
#if defined(ROLE_ANCHOR1)
static float    g_d2_m    = -1.0f;
static uint32_t g_d2_age  = 0;     // millis() момента получения
#endif

// ============================================================
//  Работа с BU04 (AT-команды)
// ============================================================

static void flushBU04() {
    while (bu04.available()) bu04.read();
}

// Отправить AT-команду, вернуть ответ. Ждём "OK"/"ERROR" или таймаут.
static String sendAT(const String &cmd, uint32_t timeoutMs = 1000) {
    flushBU04();
    bu04.println(cmd);
    String resp;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (bu04.available()) resp += (char)bu04.read();
        if (resp.indexOf("OK")    >= 0) break;
        if (resp.indexOf("ERROR") >= 0) break;
        delay(10);
    }
    return resp;
}

// Парсит первое значение distance из ответа AT+DISTANCE.
// Форматы (оба поддерживаются):
//   "distance: 0.340000"          ← одна база
//   "distance:0 0.340000\n..."    ← несколько баз (ID перед значением)
static float parseDistance(const String &resp) {
    int idx = resp.indexOf("distance:");
    if (idx < 0) return -1.0f;
    int i = idx + 9;
    // Пропускаем пробелы
    while (i < (int)resp.length() && resp[i] == ' ') i++;
    // Если стоит цифра перед пробелом — это ID якоря, пропускаем
    int j = i;
    while (j < (int)resp.length() && isdigit((uint8_t)resp[j])) j++;
    if (j < (int)resp.length() && resp[j] == ' ') i = j + 1;
    // Читаем дробное число
    String num;
    while (i < (int)resp.length() &&
           (isdigit((uint8_t)resp[i]) || resp[i] == '.')) {
        num += resp[i++];
    }
    float d = num.toFloat();
    return (d > 0.0f && d < MAX_VALID_DIST_M) ? d : -1.0f;
}

// Скользящее среднее по дистанции
static float updateAvg(float v) {
    g_avgBuf[g_avgIdx] = v;
    g_avgIdx = (g_avgIdx + 1) % MOVING_AVG_SAMPLES;
    if (g_avgIdx == 0) g_avgFull = true;
    uint8_t n = g_avgFull ? MOVING_AVG_SAMPLES : (g_avgIdx ? g_avgIdx : 1);
    float s = 0;
    for (uint8_t i = 0; i < n; i++) s += g_avgBuf[i];
    return s / n;
}

// Настройка BU04: AT+SETCFG=id,role,ch,rate → AT+SAVE (перезагрузка ~3 с)
static void configureBU04(uint8_t id, uint8_t role) {
    Serial.printf("# Настройка BU04: id=%u role=%u ch=%u rate=%u\n",
                  id, role, BU04_CHANNEL, BU04_RATE);
    String cmd = "AT+SETCFG=" + String(id)            + ","
                               + String(role)          + ","
                               + String(BU04_CHANNEL)  + ","
                               + String(BU04_RATE);
    String resp = sendAT(cmd, 1500);
    if (resp.indexOf("OK") >= 0) {
        Serial.println("# Сохранение, BU04 перезагружается (~3 с)…");
        bu04.println(AT_SAVE);
        delay(3500);
        flushBU04();
        Serial.println("# BU04 готов");
    } else {
        Serial.println("# Ошибка настройки: " + resp);
        Serial.println("# Попробуйте другой UART (PA9/PA10) или проверьте питание 500 мА");
    }
}

// ============================================================
//  ESP-NOW (только ANCHOR1 и ANCHOR2)
// ============================================================
#if defined(ROLE_ANCHOR1) || defined(ROLE_ANCHOR2)

// Пакет ESP-NOW: ANCHOR2 → ANCHOR1
struct __attribute__((packed)) EspNowPkt {
    uint16_t magic;    // ESPNOW_MAGIC – защита от чужих пакетов
    float    dist_m;   // расстояние ANCHOR2→TAG (метры)
    uint32_t ts_ms;    // millis() на момент измерения
};

#if defined(ROLE_ANCHOR2)
static uint8_t g_anchor1Mac[] = ANCHOR1_MAC;
static void onSend(const uint8_t *, esp_now_send_status_t) {}
#endif

#if defined(ROLE_ANCHOR1)
// Обработчик входящего ESP-NOW пакета
static void handlePkt(const uint8_t *, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspNowPkt)) return;
    EspNowPkt p;
    memcpy(&p, data, sizeof(p));
    if (p.magic != ESPNOW_MAGIC) return;
    g_d2_m   = p.dist_m;
    g_d2_age = millis();
}

// Враппер: совместимость Arduino-ESP32 2.x (IDF 4) и 3.x (IDF 5)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onRecv(const esp_now_recv_info_t *info,
                   const uint8_t *data, int len) {
    handlePkt(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t *mac,
                   const uint8_t *data, int len) {
    handlePkt(mac, data, len);
}
#endif
#endif  // ROLE_ANCHOR1

static void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("# ESP-NOW init ОШИБКА");
        return;
    }
#if defined(ROLE_ANCHOR1)
    Serial.printf("# MAC ANCHOR1: %s\n", WiFi.macAddress().c_str());
    Serial.println("# Скопируйте MAC в config.h → ANCHOR1_MAC, затем прошейте ANCHOR2");
    esp_now_register_recv_cb(onRecv);
#else
    esp_now_register_send_cb(onSend);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, g_anchor1Mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK)
        Serial.println("# ESP-NOW add peer ОШИБКА");
#endif
}

#endif  // ROLE_ANCHOR1 || ROLE_ANCHOR2

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    // Запускаем UART к BU04 (USART2: PA2=TX→GPIO3, PA3=RX←GPIO2)
    bu04.begin(BU04_BAUD, SERIAL_8N1, PIN_BU04_RX, PIN_BU04_TX);
    delay(2000);    // ждём загрузку STM32 в BU04
    flushBU04();

#if defined(ROLE_ANCHOR1)
    Serial.println("# ========================================");
    Serial.println("# BU04 UWB – ANCHOR1 (главный якорь)");
    Serial.println("# CSV ANCHOR1:  ANCHOR1,seq,d1_m,avg_m,ts_ms");
    Serial.println("# CSV PLATFORM: PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms");
    Serial.println("# ========================================");
    configureBU04(BU04_ID_ANCHOR1, /*role=*/1);
    initESPNow();

#elif defined(ROLE_ANCHOR2)
    Serial.println("# ========================================");
    Serial.println("# BU04 UWB – ANCHOR2 (ведомый якорь)");
    Serial.println("# CSV: ANCHOR2,seq,d2_m,ts_ms");
    Serial.println("# ========================================");
    configureBU04(BU04_ID_ANCHOR2, /*role=*/1);
    initESPNow();

#elif defined(ROLE_TAG)
    Serial.println("# ========================================");
    Serial.println("# BU04 UWB – TAG (пользователь)");
    Serial.println("# CSV: TAG,seq,d_m,avg_m,ts_ms");
    Serial.println("# IMU: IMU,<данные акселерометра BU04>,ts_ms");
    Serial.println("# ========================================");
    configureBU04(BU04_ID_TAG, /*role=*/0);
    // TODO_PDOA: если Ai-Thinker опубликует AT-команду для PDOA-угла,
    //            добавить её здесь для получения направления двумя модулями.
#endif

    Serial.println("# Запуск измерений…");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
    static uint32_t lastDist = 0;
    static uint32_t lastImu  = 0;
    uint32_t now = millis();

    // ── Периодический запрос дистанции ───────────────────────
    if (now - lastDist >= POLL_INTERVAL_MS) {
        lastDist = now;
        g_seq++;

        String resp = sendAT(AT_DISTANCE, 800);
        float  d    = parseDistance(resp);

#if defined(ROLE_ANCHOR1)
        if (d > 0.0f) {
            float avg = updateAvg(d);
            Serial.printf("ANCHOR1,%u,%.3f,%.3f,%lu\n",
                          g_seq, d, avg, now);

            // Вычисляем направление, если данные ANCHOR2 свежие
            bool d2ok = (g_d2_m > 0.0f) && ((now - g_d2_age) < ANCHOR2_STALE_MS);
            if (d2ok) {
                float B   = BASELINE_M;
                float d1  = d, d2 = g_d2_m;
                // Триангуляция: TAG в координатах платформы
                float x   = (d1*d1 - d2*d2 + B*B) / (2.0f * B);
                float d2f = d1*d1 - x*x;
                float y   = (d2f >= 0.0f) ? sqrtf(d2f) : 0.0f;
                float ang = atan2f(y, x) * 180.0f / (float)M_PI;
                Serial.printf("PLATFORM,%u,%.3f,%.3f,%.1f,%.3f,%.3f,%lu\n",
                              g_seq, d1, d2, ang, x, y, now);
            }
        }

#elif defined(ROLE_ANCHOR2)
        if (d > 0.0f) {
            Serial.printf("ANCHOR2,%u,%.3f,%lu\n", g_seq, d, now);
            // Отправляем дистанцию на ANCHOR1 через ESP-NOW
            EspNowPkt pkt = { ESPNOW_MAGIC, d, now };
            esp_now_send(g_anchor1Mac, (uint8_t *)&pkt, sizeof(pkt));
        }

#elif defined(ROLE_TAG)
        if (d > 0.0f) {
            float avg = updateAvg(d);
            Serial.printf("TAG,%u,%.3f,%.3f,%lu\n",
                          g_seq, d, avg, now);
        } else {
            Serial.printf("TAG,%u,ERROR,%lu\n", g_seq, now);
        }
#endif
    }

    // ── Опрос акселерометра (только TAG) ─────────────────────
#if defined(ROLE_TAG)
    if (now - lastImu >= IMU_INTERVAL_MS) {
        lastImu = now;
        // AT+GETSENSOR доступен только через TTL-порт BU04
        String imu = sendAT(AT_GETSENSOR, 600);
        imu.trim();
        if (imu.length() > 4 && imu.indexOf("ERROR") < 0) {
            Serial.printf("IMU,%s,%lu\n", imu.c_str(), now);
        }
    }
#endif
}
 *
 * Two roles share this source file; choose at compile time:
 *   -DROLE_ANCHOR=1   (device the user carries)
 *   -DROLE_TAG=1      (device on the mobile platform)
 *
 * Protocol: Double-Sided Two-Way Ranging (DS-TWR)
 *   TAG                          ANCHOR
 *   ───                          ──────
 *   POLL  ──────────────────────►
 *         ◄──────────────────── POLL_ACK
 *   FINAL ──────────────────────►
 *         ◄──────────────────── RANGE_REPORT
 *
 * Timestamp symbols (DW1000 40-bit, unit ≈ 15.65 ps):
 *   t1  TAG  TX of POLL
 *   t2  ANCHOR RX of POLL
 *   t3  ANCHOR TX of POLL_ACK
 *   t4  TAG  RX of POLL_ACK
 *   t5  TAG  TX of FINAL
 *   t6  ANCHOR RX of FINAL
 *
 * DS-TWR distance formula:
 *   Ra = t4 - t1    (tag round-trip, messages 1-2)
 *   Rb = t6 - t3    (anchor round-trip, messages 3-4)
 *   Da = t5 - t4    (tag reply delay)
 *   Db = t3 - t2    (anchor reply delay)
 *   ToF = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db)
 *   dist = ToF * SPEED_OF_LIGHT * DW1000_TIME_RES_S
 *
 * Serial telemetry format (CSV, 115200 baud):
 *   TAG:    TAG,<seq>,<distance_m>,<distance_mm>,<avg_m>,<ts_ms>
 *   ANCHOR: ANCHOR,<seq>,<distance_m>,<distance_mm>,<ts_ms>
 */

#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include "config.h"

// ---- compile-time role check ------------------------------
#if !defined(ROLE_ANCHOR) && !defined(ROLE_TAG)
#  error "Define either ROLE_ANCHOR=1 or ROLE_TAG=1 in build_flags"
#endif

// ---- state machine ----------------------------------------
enum class State : uint8_t {
    IDLE,
    WAIT_POLL_ACK,      // TAG  waiting for POLL_ACK
    WAIT_FINAL,         // ANCHOR waiting for FINAL
    WAIT_RANGE_REPORT,  // TAG  waiting for RANGE_REPORT
};

// ---- globals ----------------------------------------------
static volatile bool  g_rxDone    = false;
static volatile bool  g_txDone    = false;
static volatile bool  g_rxTimeout = false;
static volatile bool  g_rxError   = false;

static uint8_t  g_rxBuf[64];
static uint8_t  g_txBuf[64];
static uint8_t  g_seqNum    = 0;
static State    g_state     = State::IDLE;

// Timestamps stored across the DS-TWR exchange
// t6 is a local variable computed directly inside anchorHandleRx()
static DW1000Time g_t1, g_t2, g_t3, g_t4;

// Moving average buffer for TAG-side display
static float    g_distBuf[MOVING_AVG_SAMPLES] = {0};
static uint8_t  g_distIdx   = 0;
static bool     g_distFull  = false;

// ---- ISR callbacks ----------------------------------------
static void onRxOk()      { g_rxDone    = true; }
static void onTxOk()      { g_txDone    = true; }
static void onRxTimeout() { g_rxTimeout = true; }
static void onRxError()   { g_rxError   = true; }

// ---- helper: send a frame ---------------------------------
static void sendFrame(const uint8_t *buf, uint8_t len, bool waitForResponse = false)
{
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData(const_cast<uint8_t *>(buf), len);
    if (waitForResponse) {
        DW1000.setDelay(DW1000Time(TX_TO_RX_DELAY_US, DW1000Time::MICROSECONDS));
    }
    DW1000.startTransmit();
}

// ---- helper: start listening ------------------------------
static void startReceive(uint16_t timeoutMs = RX_TIMEOUT_MS)
{
    DW1000.newReceive();
    DW1000.setDefaults();
    if (timeoutMs > 0) {
        DW1000.setReceiveTimeout(timeoutMs);
    }
    DW1000.startReceive();
}

// ---- helper: pack/unpack DW1000Time into 5 bytes ----------
static void packTimestamp(const DW1000Time &ts, uint8_t *dst)
{
    // DW1000Time stores the raw 40-bit value
    uint64_t raw = ts.getTimestamp();
    for (int i = 0; i < 5; i++) {
        dst[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFF);
    }
}

static DW1000Time unpackTimestamp(const uint8_t *src)
{
    uint64_t raw = 0;
    for (int i = 0; i < 5; i++) {
        raw |= (static_cast<uint64_t>(src[i]) << (8 * i));
    }
    return DW1000Time(static_cast<int64_t>(raw));
}

// ---- moving average ---------------------------------------
static float updateMovingAvg(float newVal)
{
    g_distBuf[g_distIdx] = newVal;
    g_distIdx = (g_distIdx + 1) % MOVING_AVG_SAMPLES;
    if (g_distIdx == 0) g_distFull = true;

    uint8_t n = g_distFull ? MOVING_AVG_SAMPLES : g_distIdx;
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += g_distBuf[i];
    return sum / n;
}

// ============================================================
//  ANCHOR role
// ============================================================
#if defined(ROLE_ANCHOR)

static void anchorHandleRx()
{
    uint8_t len = DW1000.getDataLength();
    DW1000.getData(g_rxBuf, len);

    if (g_state == State::IDLE) {
        // Expect POLL
        if (len < FRAME_LEN_POLL || g_rxBuf[0] != MSG_POLL) {
            startReceive();
            return;
        }
        // Record t2 (our RX time of POLL)
        DW1000.getReceiveTimestamp(g_t2);

        // Build POLL_ACK
        g_txBuf[0] = MSG_POLL_ACK;
        g_txBuf[1] = g_rxBuf[1];   // echo sequence number

        g_state = State::WAIT_FINAL;
        // Send POLL_ACK; enable auto-RX after transmit
        sendFrame(g_txBuf, FRAME_LEN_POLL_ACK, /*waitForResponse=*/true);
        // t3 captured after TX done (see anchorHandleTx)

    } else if (g_state == State::WAIT_FINAL) {
        // Expect FINAL
        if (len < FRAME_LEN_FINAL || g_rxBuf[0] != MSG_FINAL) {
            g_state = State::IDLE;
            startReceive();
            return;
        }
        DW1000Time t6;
        DW1000.getReceiveTimestamp(t6);     // our RX of FINAL

        // Unpack TAG's timestamps from FINAL payload
        DW1000Time t1 = unpackTimestamp(&g_rxBuf[2]);       // TAG TX POLL
        DW1000Time t4 = unpackTimestamp(&g_rxBuf[7]);       // TAG RX POLL_ACK
        DW1000Time t5 = unpackTimestamp(&g_rxBuf[12]);      // TAG TX FINAL

        // DS-TWR calculation
        DW1000Time Ra = t4 - t1;    // tag round-trip (poll+ack)
        DW1000Time Rb = t6 - g_t3; // anchor round-trip (final)
        DW1000Time Da = t5 - t4;    // tag reply delay
        DW1000Time Db = g_t3 - g_t2; // anchor reply delay

        double Ra_d = static_cast<double>(Ra.getTimestamp());
        double Rb_d = static_cast<double>(Rb.getTimestamp());
        double Da_d = static_cast<double>(Da.getTimestamp());
        double Db_d = static_cast<double>(Db.getTimestamp());

        double denom = Ra_d + Rb_d + Da_d + Db_d;
        double distM = 0.0;
        if (denom > 0) {
            double tof_ticks = (Ra_d * Rb_d - Da_d * Db_d) / denom;
            distM = tof_ticks * DW1000_TIME_RES_S * SPEED_OF_LIGHT;
        }

        // Sanity check
        if (distM < 0 || distM > MAX_VALID_DISTANCE_M) distM = -1.0;

        uint32_t distMm = (distM >= 0) ? static_cast<uint32_t>(distM * 1000.0) : 0xFFFFFFFF;

        // Build RANGE_REPORT
        g_txBuf[0] = MSG_RANGE_REPORT;
        g_txBuf[1] = g_rxBuf[1];
        g_txBuf[2] = (distMm)       & 0xFF;
        g_txBuf[3] = (distMm >> 8)  & 0xFF;
        g_txBuf[4] = (distMm >> 16) & 0xFF;
        g_txBuf[5] = (distMm >> 24) & 0xFF;

        sendFrame(g_txBuf, FRAME_LEN_RANGE_REPORT);
        g_state = State::IDLE;

        // Print telemetry
        if (distM >= 0) {
            Serial.printf("ANCHOR,%u,%.3f,%lu,%lu\n",
                           g_rxBuf[1], distM, (unsigned long)distMm, millis());
        }
    }
}

static void anchorHandleTx()
{
    if (g_state == State::WAIT_FINAL) {
        // Capture t3: our TX timestamp of POLL_ACK
        DW1000.getTransmitTimestamp(g_t3);
        // Auto-RX should already be active from waitForResponse flag,
        // but set a timeout guard just in case
        DW1000.newReceive();
        DW1000.setDefaults();
        DW1000.setReceiveTimeout(RX_TIMEOUT_MS);
        DW1000.startReceive();
    } else if (g_state == State::IDLE) {
        // RANGE_REPORT sent; go back to listening for next POLL
        startReceive();
    }
}

#endif // ROLE_ANCHOR

// ============================================================
//  TAG role
// ============================================================
#if defined(ROLE_TAG)

static void tagSendPoll()
{
    g_seqNum++;
    g_txBuf[0] = MSG_POLL;
    g_txBuf[1] = g_seqNum;
    g_state = State::WAIT_POLL_ACK;
    sendFrame(g_txBuf, FRAME_LEN_POLL, /*waitForResponse=*/true);
    // t1 captured in tagHandleTx
}

static void tagHandleRx()
{
    uint8_t len = DW1000.getDataLength();
    DW1000.getData(g_rxBuf, len);

    if (g_state == State::WAIT_POLL_ACK) {
        if (len < FRAME_LEN_POLL_ACK || g_rxBuf[0] != MSG_POLL_ACK
                || g_rxBuf[1] != g_seqNum) {
            // Unexpected – restart
            g_state = State::IDLE;
            return;
        }
        DW1000.getReceiveTimestamp(g_t4);   // t4: our RX of POLL_ACK

        // Build FINAL with timestamps t1, t4, t5_estimated
        // We use a pre-computed TX delay so t5 is known before transmit
        // Encode t1 (TAG TX POLL), t4 (TAG RX POLL_ACK); t5 packed after transmit
        // For simplicity, we pack a zero placeholder for t5 and send immediately;
        // the library fills in the actual TX timestamp which we then correct.
        // Simpler approach: schedule FINAL with a fixed delay and compute t5.
        DW1000Time replyDelay(TX_TO_RX_DELAY_US * 2, DW1000Time::MICROSECONDS);
        DW1000Time t5 = g_t4 + replyDelay;

        g_txBuf[0] = MSG_FINAL;
        g_txBuf[1] = g_seqNum;
        packTimestamp(g_t1, &g_txBuf[2]);
        packTimestamp(g_t4, &g_txBuf[7]);
        packTimestamp(t5,   &g_txBuf[12]);

        g_state = State::WAIT_RANGE_REPORT;

        DW1000.newTransmit();
        DW1000.setDefaults();
        DW1000.setData(g_txBuf, FRAME_LEN_FINAL);
        DW1000.setDelay(replyDelay);    // schedule TX at t5
        DW1000.startTransmit();

    } else if (g_state == State::WAIT_RANGE_REPORT) {
        if (len < FRAME_LEN_RANGE_REPORT || g_rxBuf[0] != MSG_RANGE_REPORT
                || g_rxBuf[1] != g_seqNum) {
            g_state = State::IDLE;
            return;
        }
        uint32_t distMm =  ((uint32_t)g_rxBuf[2])
                         | ((uint32_t)g_rxBuf[3] << 8)
                         | ((uint32_t)g_rxBuf[4] << 16)
                         | ((uint32_t)g_rxBuf[5] << 24);

        float distM = (distMm != 0xFFFFFFFF)
                      ? static_cast<float>(distMm) / 1000.0f
                      : -1.0f;

        float avgM = (distM >= 0) ? updateMovingAvg(distM) : -1.0f;

        if (distM >= 0) {
            Serial.printf("TAG,%u,%.3f,%lu,%.3f,%lu\n",
                           g_seqNum, distM, (unsigned long)distMm, avgM, millis());
        } else {
            Serial.printf("TAG,%u,ERROR,0,0,%lu\n", g_seqNum, millis());
        }
        g_state = State::IDLE;
    }
}

static void tagHandleTx()
{
    if (g_state == State::WAIT_POLL_ACK) {
        // Capture t1: TAG TX timestamp of POLL
        DW1000.getTransmitTimestamp(g_t1);
        // Auto-RX should be active; set a timeout guard
        DW1000.newReceive();
        DW1000.setDefaults();
        DW1000.setReceiveTimeout(RX_TIMEOUT_MS);
        DW1000.startReceive();
    } else if (g_state == State::WAIT_RANGE_REPORT) {
        // FINAL sent; switch to RX for RANGE_REPORT
        DW1000.newReceive();
        DW1000.setDefaults();
        DW1000.setReceiveTimeout(RX_TIMEOUT_MS);
        DW1000.startReceive();
    }
}

#endif // ROLE_TAG

// ============================================================
//  setup()
// ============================================================
void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

#if defined(ROLE_ANCHOR)
    Serial.println("# BU04 UWB – ANCHOR role starting");
    Serial.println("# CSV: ANCHOR,seq,distance_m,distance_mm,timestamp_ms");
#else
    Serial.println("# BU04 UWB – TAG role starting");
    Serial.println("# CSV: TAG,seq,distance_m,distance_mm,avg_m,timestamp_ms");
#endif

    // Initialise DW1000 driver
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_DW_CS);
    DW1000.begin(PIN_DW_IRQ, PIN_DW_RST);
    DW1000.select(PIN_DW_CS);

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(
#if defined(ROLE_ANCHOR)
        1
#else
        2
#endif
    );
    DW1000.setNetworkId(0xDECA);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();

    // Register ISR callbacks
    DW1000.attachReceivedHandler(onRxOk);
    DW1000.attachTransmitHandler(onTxOk);
    DW1000.attachReceiveTimeoutHandler(onRxTimeout);
    DW1000.attachReceiveErrorHandler(onRxError);

#if defined(ROLE_ANCHOR)
    // Anchor starts by listening
    startReceive(0);   // 0 = infinite timeout (keeps looping)
    g_state = State::IDLE;
    Serial.println("# Anchor ready, waiting for TAG poll …");
#else
    // Tag will start transmitting in loop()
    g_state = State::IDLE;
    Serial.println("# Tag ready, starting ranging …");
#endif
}

// ============================================================
//  loop()
// ============================================================
void loop()
{
    // ---- Handle DW1000 interrupts ----
    if (g_rxDone) {
        g_rxDone = false;
#if defined(ROLE_ANCHOR)
        anchorHandleRx();
#else
        tagHandleRx();
#endif
    }

    if (g_txDone) {
        g_txDone = false;
#if defined(ROLE_ANCHOR)
        anchorHandleTx();
#else
        tagHandleTx();
#endif
    }

    if (g_rxTimeout || g_rxError) {
        if (g_rxError) {
            Serial.printf("# RX error – seq %u\n", g_seqNum);
        }
        g_rxTimeout = false;
        g_rxError   = false;
        g_state = State::IDLE;

#if defined(ROLE_ANCHOR)
        // Return to idle RX
        startReceive(0);
#endif
    }

    // ---- TAG: kick off a new ranging round periodically ----
#if defined(ROLE_TAG)
    static uint32_t lastRangingMs = 0;
    if (g_state == State::IDLE) {
        uint32_t now = millis();
        if (now - lastRangingMs >= RANGING_INTERVAL_MS) {
            lastRangingMs = now;
            tagSendPoll();
        }
    }
#endif
}
