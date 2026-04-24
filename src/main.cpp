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
 * Источник AT-команд: BU03/BU04 AT指令 V1.0.6
 *
 * ───────────── Режимы UWB ─────────────────────────────────
 *
 *  TWR (по умолчанию, #define PDOA_MODE НЕ задан):
 *    Два якоря на платформе + тег. Каждый якорь измеряет AT+DISTANCE
 *    до тега. ANCHOR2 шлёт d2 на ANCHOR1 по ESP-NOW. ANCHOR1 считает
 *    координаты x/y и угол β методом трилатерации.
 *
 *    ANCHOR1(0,0) ──── B ──── ANCHOR2(B,0)
 *                TAG(x, y)
 *    x = (d1² − d2² + B²) / (2·B)
 *    y = √(d1² − x²)
 *    β = atan2(y, x)   ← угол от оси ANCHOR1→ANCHOR2
 *
 *  PDOA (#define PDOA_MODE задан):
 *    Один якорь с двумя антеннами измеряет угол + расстояние до тега
 *    с помощью фазовой разности (Phase Difference of Arrival).
 *    ANCHOR2 и ESP-NOW не нужны. Якорь работает в режиме AT+SETUWBMODE=1,
 *    данные поступают как JSON-пакеты (AT+USER_CMD=0) при регистрации
 *    тега командой AT+ADDTAG.
 *
 * ───────────── Ориентация тега ────────────────────────────
 *   Пользователь держит тег произвольно. UWB измеряет расстояние
 *   по времени полёта (ToF) — ориентация антенны не влияет на
 *   точность дистанции при нормальном SNR. PDOA-угол измеряется
 *   в плоскости двух антенн якоря и не зависит от ориентации тега.
 *
 * ───────────── Формат телеметрии (CSV, 115200 бод) ─────────
 *  TWR:
 *   ANCHOR1,seq,d1_m,avg_m,ts_ms
 *   ANCHOR2,seq,d2_m,ts_ms
 *   PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms
 *   TAG,seq,d_m,avg_m,ts_ms
 *  PDOA:
 *   PDOA,seq,dist_m,angle_deg,ts_ms
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

// Настройка BU04: AT+SETCFG=ID,Role,CH,Rate,Group → AT+SAVE (перезагрузка ~3 с)
// Источник: BU03/BU04 AT指令 V1.0.6, раздел 1
// Rate всегда 1 (6.8 Мбит/с — единственный поддерживаемый вариант).
// Group: якоря настраивают в свою группу; теги оставляют 0.
static void configureBU04(uint8_t id, uint8_t role) {
    uint8_t group = (role == 1) ? BU04_GROUP : 0;
    Serial.printf("# Настройка BU04: id=%u role=%u ch=%u rate=%u group=%u\n",
                  id, role, BU04_CHANNEL, BU04_RATE, group);
    String cmd = "AT+SETCFG=" + String(id)           + ","
                               + String(role)         + ","
                               + String(BU04_CHANNEL) + ","
                               + String(BU04_RATE)    + ","
                               + String(group);
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
#if defined(PDOA_MODE)
    Serial.println("# BU04 UWB – ANCHOR1 (PDOA режим, 1 якорь)");
    Serial.println("# CSV: PDOA,seq,dist_m,angle_deg,ts_ms");
#else
    Serial.println("# BU04 UWB – ANCHOR1 (TWR режим, главный якорь)");
    Serial.println("# CSV ANCHOR1:  ANCHOR1,seq,d1_m,avg_m,ts_ms");
    Serial.println("# CSV PLATFORM: PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms");
#endif
    Serial.println("# ========================================");
    configureBU04(BU04_ID_ANCHOR1, /*role=*/1);

#if defined(PDOA_MODE)
    // Переключаем BU04 в режим PDOA (AT+SETUWBMODE=1), если ещё не был
    {
        String mode = sendAT(AT_GETUWBMODE, 600);
        if (mode.indexOf('1') < 0) {
            Serial.println("# Переключение BU04 в режим PDOA…");
            String r = sendAT(AT_SETUWBMODE_PDOA, 1000);
            if (r.indexOf("OK") >= 0) {
                bu04.println(AT_SAVE);
                delay(3500); flushBU04();
                Serial.println("# PDOA включён");
            } else {
                Serial.println("# Ошибка переключения PDOA: " + r);
            }
        } else {
            Serial.println("# BU04 уже в режиме PDOA");
        }
        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
        Serial.println("# Формат вывода: JSON");
        Serial.println("# Добавьте тег: AT+ADDTAG=<LongAddr64>,<ShortAddr>,1,64,0");
    }
#else
    initESPNow();
#endif

#elif defined(ROLE_ANCHOR2)
    Serial.println("# ========================================");
    Serial.println("# BU04 UWB – ANCHOR2 (TWR режим, ведомый якорь)");
    Serial.println("# CSV: ANCHOR2,seq,d2_m,ts_ms");
    Serial.println("# ========================================");
    configureBU04(BU04_ID_ANCHOR2, /*role=*/1);
    initESPNow();

#elif defined(ROLE_TAG)
    Serial.println("# ========================================");
    Serial.println("# BU04 UWB – TAG (пользователь)");
    Serial.println("# CSV: TAG,seq,d_m,avg_m,ts_ms");
    // Примечание: AT+GETSENSOR поддерживает только BU03.
    // BU04 акселерометра не имеет (нет аппаратной поддержки).
    Serial.println("# ========================================");
    configureBU04(BU04_ID_TAG, /*role=*/0);
#endif

    Serial.println("# Запуск измерений…");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
    static uint32_t lastDist = 0;
    uint32_t now = millis();

    // ── Периодический запрос дистанции ───────────────────────
    if (now - lastDist >= POLL_INTERVAL_MS) {
        lastDist = now;
        g_seq++;

#if defined(ROLE_ANCHOR1) && defined(PDOA_MODE)
        // PDOA режим: BU04 непрерывно шлёт строки с данными, когда видит
        // зарегистрированный тег.
        //
        // Реальный формат (из официального GUI Ai_Thinker_PDOA_V1_0_1):
        //   Tag_Addr:XXXX, Seq:N, Xcm:X.XX, Ycm:X.XX, Range:X.XX, Angle:N
        //   Tag_Addr — короткий адрес тега (16-бит hex)
        //   Xcm/Ycm — позиция тега в см в системе координат якоря
        //   Range   — дистанция в см (округлённо)
        //   Angle   — угол в градусах (целое)
        //
        // Читаем всё, что накопилось в буфере UART за период POLL_INTERVAL_MS.
        // Одна строка = одно измерение. Если строк несколько — берём последнюю.
        String lastLine;
        {
            String buf;
            uint32_t t0 = millis();
            while (millis() - t0 < (POLL_INTERVAL_MS - 10)) {
                while (bu04.available()) {
                    char c = (char)bu04.read();
                    if (c == '\n') {
                        buf.trim();
                        if (buf.startsWith("Tag_Addr:")) lastLine = buf;
                        buf = "";
                    } else {
                        buf += c;
                    }
                }
                if (lastLine.length() > 0 && !bu04.available()) break;
                delay(2);
            }
        }
        if (lastLine.length() > 0) {
            // Парсим: Tag_Addr:XXXX, Seq:N, Xcm:V, Ycm:V, Range:V, Angle:V
            uint32_t addr  = 0;
            int      seq   = 0;
            float    xcm   = 0, ycm = 0, range = 0;
            int      angle = 0;
            // sscanf не доступен надёжно на Arduino; парсим вручную через indexOf
            auto fieldVal = [&](const char *key) -> String {
                int i = lastLine.indexOf(key);
                if (i < 0) return "";
                i += strlen(key);
                int j = lastLine.indexOf(',', i);
                return (j >= 0) ? lastLine.substring(i, j) : lastLine.substring(i);
            };
            String sAddr  = fieldVal("Tag_Addr:");
            String sSeq   = fieldVal("Seq:");
            String sXcm   = fieldVal("Xcm:");
            String sYcm   = fieldVal("Ycm:");
            String sRange = fieldVal("Range:");
            String sAngle = fieldVal("Angle:");
            if (sAddr.length())  addr  = (uint32_t)strtoul(sAddr.c_str(), nullptr, 16);
            if (sSeq.length())   seq   = sSeq.toInt();
            if (sXcm.length())   xcm   = sXcm.toFloat();
            if (sYcm.length())   ycm   = sYcm.toFloat();
            if (sRange.length()) range = sRange.toFloat();
            if (sAngle.length()) angle = sAngle.toInt();
            // Вывод CSV: PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms
            Serial.printf("PDOA,%04X,%d,%.2f,%d,%.2f,%.2f,%lu\n",
                          addr, seq,
                          range / 100.0f, angle,
                          xcm  / 100.0f,
                          ycm  / 100.0f,
                          now);
        }

#else  // TWR режим (ANCHOR1, ANCHOR2, TAG)
        String resp = sendAT(AT_DISTANCE, 800);
        float  d    = parseDistance(resp);

#  if defined(ROLE_ANCHOR1)
        if (d > 0.0f) {
            float avg = updateAvg(d);
            Serial.printf("ANCHOR1,%u,%.3f,%.3f,%lu\n",
                          g_seq, d, avg, now);

            // Вычисляем направление, если данные ANCHOR2 свежие
            bool d2ok = (g_d2_m > 0.0f) && ((now - g_d2_age) < ANCHOR2_STALE_MS);
            if (d2ok) {
                float B   = BASELINE_M;
                float d1  = d, d2 = g_d2_m;
                // Трилатерация: TAG в системе координат платформы
                float x   = (d1*d1 - d2*d2 + B*B) / (2.0f * B);
                float d2f = d1*d1 - x*x;
                float y   = (d2f >= 0.0f) ? sqrtf(d2f) : 0.0f;
                float ang = atan2f(y, x) * 180.0f / (float)M_PI;
                Serial.printf("PLATFORM,%u,%.3f,%.3f,%.1f,%.3f,%.3f,%lu\n",
                              g_seq, d1, d2, ang, x, y, now);
            }
        }

#  elif defined(ROLE_ANCHOR2)
        if (d > 0.0f) {
            Serial.printf("ANCHOR2,%u,%.3f,%lu\n", g_seq, d, now);
            // Отправляем дистанцию на ANCHOR1 через ESP-NOW
            EspNowPkt pkt = { ESPNOW_MAGIC, d, now };
            esp_now_send(g_anchor1Mac, (uint8_t *)&pkt, sizeof(pkt));
        }

#  elif defined(ROLE_TAG)
        if (d > 0.0f) {
            float avg = updateAvg(d);
            Serial.printf("TAG,%u,%.3f,%.3f,%lu\n",
                          g_seq, d, avg, now);
        } else {
            Serial.printf("TAG,%u,ERROR,%lu\n", g_seq, now);
        }
#  endif  // role

#endif  // TWR/PDOA
    }
}
