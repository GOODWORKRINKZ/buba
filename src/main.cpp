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

// Проверка что MAC не заглушка (FF:FF:FF:FF:FF:FF)
static bool isMacDefault() {
    for (int i = 0; i < 6; i++) if (g_anchor1Mac[i] != 0xFF) return false;
    return true;
}
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
    if (isMacDefault()) {
        Serial.println("# ВНИМАНИЕ: ANCHOR1_MAC не задан! Замените в config.h и перепрошейте ANCHOR2.");
        Serial.println("# ESP-NOW отправка не работает с MAC FF:FF:FF:FF:FF:FF.");
    }
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
