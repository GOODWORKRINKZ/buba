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
 *   PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms
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

// Ждём готовности BU04 — пингуем AT до ответа OK (до 15 с)
static bool waitBU04Ready(const char *label = nullptr) {
    if (label) { Serial.print(label); Serial.print(" "); }
    Serial.print("# Ожидание BU04");
    bool ready = false;
    for (int i = 0; i < 30 && !ready; i++) {
        flushBU04();
        bu04.println("AT");
        uint32_t t0 = millis();
        String r;
        while (millis() - t0 < 500) {
            while (bu04.available()) r += (char)bu04.read();
            if (r.indexOf("OK") >= 0) { ready = true; break; }
            delay(10);
        }
        if (!ready) { Serial.print('.'); delay(500); }
    }
    Serial.println(ready ? " OK" : " TIMEOUT");
    flushBU04();
    return ready;
}

// Ждём готовности BU04 бесконечно, с периодическим printout.
// Возвращает управление только когда BU04 ответил OK.
static void waitBU04ReadyForever(const char *msg) {
    Serial.println(msg);
    while (true) {
        if (waitBU04Ready()) return;
        Serial.println("# BU04 не отвечает — передёрните питание ТОЛЬКО BU04 (не ESP32)");
        delay(2000);
    }
}

// Отправить AT-команду, вернуть ответ. Ждём "OK"/"ERR" или таймаут.
static String sendAT(const String &cmd, uint32_t timeoutMs = 1000) {
    flushBU04();
    bu04.println(cmd);
    String resp;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (bu04.available()) resp += (char)bu04.read();
        if (resp.indexOf("OK")  >= 0) break;
        if (resp.indexOf("ERR") >= 0) break;  // BU04 шлёт "ERR", не "ERROR"
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

// f_getcfg (cmd_fn.c) всегда возвращает "getcfg ID:X, Role:X, CH:X, Rate:X"
// независимо от twr_pdoa_mode.
//
// f_setcfg для role=0 ВСЕГДА вызывает tag_start() без reset_DWIC → INIT FAILED навсегда
// при холодном старте. at_cmd_recv (workmode=1) не понимает AT+SETCFG — другой протокол.
//
// Стратегия для role=0 (TAG): 2-фазная через PDOA-якорь.
//   Фаза 1 (ID=65535): burst SETCFG=id,1 + SETUWBMODE=1 + SAVE
//     → node_start() → reset_DWIC → DW3000 up → event-loop обрабатывает SETUWBMODE+SAVE
//     → reboot → ID=id, role=1, twr_pdoa_mode=1 → ds_twr_sts_sdc_responder()
//   Фаза 2 (ID=id, Role=1): burst SETCFG=id,0 + SAVE
//     → tag_start() из event-loop PDOA-якоря — DW3000 уже работает
//     → dwt_initialise() OK → tag_start() входит в цикл → SAVE обрабатывается
//     → reboot → ID=id, role=0, twr_pdoa_mode=1 → ds_twr_sts_sdc_initiator() ✓
//
// Стратегия для role=1 (ANCHOR TWR):
//   Burst SETCFG+SAVE → node_start() (с reset_DWIC) → SAVE через UART ISR → OK.

// Ожидает пока BU04 перестанет отвечать (уйдёт на перезагрузку).
// Возвращает true если BU04 "погас" в течение timeoutMs.
static bool waitBU04Dark(uint32_t timeoutMs) {
    Serial.print("# Ожидание сброса BU04");
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        flushBU04();
        bu04.print("AT\r\n");
        delay(250);
        bool alive = bu04.available();
        flushBU04();
        if (!alive) { Serial.println(" (сброс!)"); return true; }
        Serial.print('.');
        delay(100);
    }
    Serial.println(" (не перезагрузился!)");
    return false;
}

static void configureBU04(uint8_t id, uint8_t role) {
    const String wantId   = "ID:"   + String(id)   + ",";
    const String wantRole = "Role:" + String(role) + ",";
    for (;;) {
        String cur = sendAT(AT_GETCFG, 2000);
        { int nl = cur.indexOf('\n'); Serial.printf("# GETCFG: %s\n", (nl > 0 ? cur.substring(0, nl) : cur).c_str()); }

        // BU04 в INIT FAILED — DW3000 ещё не поднялся, ждём
        if (cur.indexOf("INIT FAILED") >= 0) {
            Serial.println("# BU04 инициализируется (INIT FAILED), ждём 3 с…");
            delay(3000);
            continue;
        }

        if (cur.indexOf(wantId) >= 0 && cur.indexOf(wantRole) >= 0) {
            Serial.printf("# BU04 конфиг: id=%u role=%u\n", id, role);
            return;
        }

        if (role == 0) {
            // ── TAG: настройка через PDOA-режим ──────────────────────────
            // Проблема: SETCFG → tag_start() → INIT FAILED → while(1){}.
            // SAVE не обрабатывается, watchdog сбрасывает BU04 → заводские.
            //
            // Стратегия: сначала переводим в PDOA (SETUWBMODE+SAVE, без SETCFG).
            // BU04 с twr_pdoa_mode=1 + nodeAddr=0xFFFF → AT-цикл + retry DW3000.
            // Ждём пока DW3000 поднимется → SETCFG БЕЗ SAVE (только RAM).
            // Если DW3000 жив → SETCFG применился → SAVE → готово.
            // Если DW3000 мёртв → tag_start() → while(1) → watchdog → рестарт → повтор.

            for (;;) {
                String cur = sendAT(AT_GETCFG, 2000);
                { int nl = cur.indexOf('\n'); Serial.printf("# GETCFG: %s\n", (nl > 0 ? cur.substring(0, nl) : cur).c_str()); }

                // Уже тег с нужным ID — выходим
                if (cur.indexOf(wantId) >= 0 && cur.indexOf(wantRole) >= 0) {
                    Serial.printf("# BU04 конфиг: id=%u role=%u\n", id, role);
                    return;
                }

                // Проверяем режим
                String uwbm = sendAT(AT_GETUWBMODE, 600);
                bool inPdoa = uwbm.indexOf("1") >= 0;

                if (!inPdoa) {
                    // Шаг 1: вход в PDOA БЕЗ SETCFG
                    Serial.println("# [TAG] Вход в PDOA-режим (SETUWBMODE+SAVE)...");
                    flushBU04();
                    bu04.print("AT+SETUWBMODE=1\r\n");
                    bu04.print("AT+SAVE\r\n");
                    waitBU04ReadyForever("# Ждём перезагрузку BU04...");
                    delay(5000);
                    flushBU04();
                    continue;
                }

                // Шаг 2: PDOA-режим есть, ждём DW3000.
                // Пробуем SETCFG БЕЗ SAVE — только в RAM.
                Serial.println("# [TAG] Попытка SETCFG (без SAVE, только RAM)...");
                flushBU04();
                bu04.print("AT+SETCFG=" + String(id) + ",0," +
                           String(BU04_CHANNEL) + "," + String(BU04_RATE) + "\r\n");
                // Ждём: либо tag_start() отработал, либо watchdog сбросил
                delay(5000);
                flushBU04();

                // Проверяем — применился ли SETCFG?
                cur = sendAT(AT_GETCFG, 2000);
                { int nl = cur.indexOf('\n'); Serial.printf("# GETCFG: %s\n", (nl > 0 ? cur.substring(0, nl) : cur).c_str()); }

                if (cur.indexOf(wantId) >= 0 && cur.indexOf(wantRole) >= 0) {
                    // DW3000 жив! Сохраняем.
                    Serial.println("# DW3000 работает! Сохраняем конфиг...");
                    sendAT(AT_SAVE, 500);
                    waitBU04ReadyForever("# Ждём перезагрузку после SAVE...");
                    delay(3000);
                    flushBU04();
                    continue;  // проверим GETCFG на след. итерации
                }

                // BU04 вернулся к заводским — DW3000 не поднялся.
                // Ждём и пробуем снова.
                Serial.println("# DW3000 пока не готов, ждём 5 с…");
                delay(5000);
            }

        } else {
            // ── ANCHOR TWR: burst SETCFG+SAVE ───────────────────────────────────
            // node_start() вызывает reset_DWIC → DW3000 init OK.
            // SAVE через UART ISR очередь → NVM записан → NVIC_SystemReset.
            Serial.println("# [ANCHOR] Запись конфига (SETCFG+SAVE)...");
            flushBU04();
            bu04.print("AT+SETCFG=" + String(id) + ",1," +
                       String(BU04_CHANNEL) + "," + String(BU04_RATE) + "\r\n");
            bu04.print("AT+SAVE\r\n");
            Serial.println("# Ждём 12с (node_start + SAVE + reset)...");
            delay(12000);
            Serial.println("# Нужен power cycle. Выключите и включите ТОЛЬКО BU04.");
            Serial.print("# Ожидание отключения BU04");
            while (true) {
                flushBU04();
                bu04.print("AT\r\n");
                uint32_t t0 = millis();
                bool alive = false;
                while (millis() - t0 < 300) {
                    if (bu04.available()) { alive = true; flushBU04(); break; }
                    delay(5);
                }
                if (!alive) { Serial.println(" (выключен)"); break; }
                Serial.print('.');
                delay(200);
            }
            flushBU04();
            Serial.println("# Включите питание BU04");
            Serial.print("# Ожидание старта BU04");
            {
                uint32_t deadline = millis() + 60000;
                while (millis() < deadline && !bu04.available()) {
                    if (millis() % 2000 < 5) Serial.print('.');
                    delay(5);
                }
                Serial.println(bu04.available() ? " (старт!)" : " (таймаут)");
            }
            flushBU04();
            Serial.println("# Ждём 8с (node_start + инициализация DW3000)...");
            delay(8000);
        }
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

// Регистрирует тег в BU04 через AT+GETDLIST / AT+ADDTAG / AT+SAVE.
// Идемпотентна: если KList > 0 — тег уже зарегистрирован, ничего не делает (D-07).
// Ждёт бесконечно, кормит WDT через delay(2000) (D-08).
static void getdlistAndRegister() {
    // Проверяем, не зарегистрирован ли тег уже (idempotent guard, D-07)
    {
        String klist = sendAT(AT_GETKLIST, 1000);
        // "getklist TagNum:N" → если N > 0, тег уже есть
        int idx = klist.indexOf("TagNum:");
        if (idx >= 0) {
            int n = klist.substring(idx + 7).toInt();
            if (n > 0) {
                Serial.printf("# Тег уже зарегистрирован (KList=%d), пропускаем AT+ADDTAG\n", n);
                return;
            }
        }
    }

    // Ждём появления тега в эфире (D-04)
    Serial.println("# Ожидание тега (AT+GETDLIST)...");
    String longAddr;
    for (int attempt = 1; ; attempt++) {
        Serial.printf("# Ожидание тега... [%d] попытка\n", attempt);
        String dlist = sendAT(AT_GETDLIST, 1500);

        // Парсим LongAddr64 из ответа (формат: "LongAddr64:XXXXXXXXXXXXXXXX ...")
        int la = dlist.indexOf("LongAddr64:");
        if (la >= 0) {
            // Извлекаем 16 hex-символов после "LongAddr64:"
            String rest = dlist.substring(la + 11);
            rest.trim();
            // Адрес — до первого пробела или конца строки
            int sp = rest.indexOf(' ');
            longAddr = (sp >= 0) ? rest.substring(0, sp) : rest.substring(0, 16);
            longAddr.trim();
            if (longAddr.length() >= 8) break;  // адрес найден
        }
        delay(2000);  // кормим WDT (D-08); vTaskDelay внутри сбрасывает TWDT
    }
    Serial.printf("# Тег обнаружен: LongAddr64=%s\n", longAddr.c_str());

    // Регистрируем тег (D-05)
    // AT+ADDTAG=<LongAddr64>,<ShortAddr>,<MinRate>,<MaxRate>,<Mode>
    // MinRate=1, MaxRate=64, Mode=0 — стандартные параметры PDOA
    char addtagCmd[64];
    snprintf(addtagCmd, sizeof(addtagCmd), "AT+ADDTAG=%s,%04X,1,64,0",
             longAddr.c_str(), (unsigned)TAG_SHORT_ADDR);
    String addResp = sendAT(String(addtagCmd), 2000);
    if (addResp.indexOf("OK") < 0) {
        Serial.printf("# AT+ADDTAG ОШИБКА: %s\n", addResp.c_str());
    } else {
        Serial.printf("# AT+ADDTAG OK (тег %04X)\n", (unsigned)TAG_SHORT_ADDR);
    }

    // Верификация (D-05)
    String klist2 = sendAT(AT_GETKLIST, 1000);
    Serial.printf("# GETKLIST: %s\n", klist2.c_str());

    // Сохраняем в NVM и ждём перезагрузки BU04 (D-05, D-06, D-10)
    Serial.println("# AT+SAVE → BU04 перезагружается...");
    sendAT(AT_SAVE, 500);  // BU04 уходит на NVIC_SystemReset сразу
    delay(500);
    waitBU04ReadyForever("# Ждём перезагрузку BU04 после AT+SAVE...");
    delay(3000);
    flushBU04();
    Serial.println("# BU04 готов после AT+SAVE");
}

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    // Запускаем UART к BU04 (USART1: PA9=TX→GPIO3, PA10=RX←GPIO2)
    bu04.begin(BU04_BAUD, SERIAL_8N1, PIN_BU04_RX, PIN_BU04_TX);
    delay(500);

    // Ждём готовности BU04 — пингуем AT до ответа OK
    waitBU04Ready();
    // BU04 отвечает на AT раньше, чем заканчивает читать конфиг из flash.
    // Даём 3 секунды на полную инициализацию DW3000 и загрузку NVM.
    Serial.println("# Ждём инициализацию BU04 (3 с)…");
    delay(3000);
    flushBU04();

#if defined(ROLE_ANCHOR1)
    Serial.println("# ========================================");
#if defined(PDOA_MODE)
    Serial.println("# BU04 UWB – ANCHOR1 (PDOA режим, 1 якорь)");
    Serial.println("# CSV: PDOA,addr_hex,seq,range_m,angle_deg,x_m,y_m,ts_ms");
#else
    Serial.println("# BU04 UWB – ANCHOR1 (TWR режим, главный якорь)");
    Serial.println("# CSV ANCHOR1:  ANCHOR1,seq,d1_m,avg_m,ts_ms");
    Serial.println("# CSV PLATFORM: PLATFORM,seq,d1_m,d2_m,angle_deg,x_m,y_m,ts_ms");
#endif
    Serial.println("# ========================================");

#if defined(PDOA_MODE)
    // ── Конфигурация BU04 для PDOA ──────────────────────────────────────────
    // Из SDK (cmd_fn.c):
    //   AT+SETCFG     → RAM + вызывает node_start() → возможен INIT FAILED
    //   AT+SETUWBMODE → RAM только (twr_pdoa_mode), нет ответа, нет сброса
    //   AT+SAVE       → пишет весь sys_para в NVM + NVIC_SystemReset
    //
    // Ключевой факт: при загрузке с twr_pdoa_mode=1 BU04 идёт в PDOA-ветку
    // (ds_twr_sts_sdc_responder), а не в node_start(). DW3000 инициализируется
    // чисто → нет INIT FAILED.
    //
    // Стратегия: бурст SETCFG + SETUWBMODE=1 + SAVE в одном потоке.
    // node_start() может заблокировать UART на ~4с, но SAVE уже в буфере
    // STM32 UART RX. После NVIC_SystemReset → загрузка в PDOA → всё чисто.
    {
        // Сначала проверяем, не в PDOA ли уже (twr_pdoa_mode=1 → всё сохранено)
        String uwbm = sendAT(AT_GETUWBMODE, 600);
        bool inPdoa = uwbm.indexOf("1") >= 0;

        if (!inPdoa) {
            // TWR режим — проверяем ID/Role в TWR-формате AT+GETCFG
            // В PDOA-режиме AT+GETCFG имеет другой формат (AncID, Dlist, …)
            String cfg = sendAT(AT_GETCFG, 2000);
            bool twrOk = cfg.indexOf("ID:" + String(BU04_ID_ANCHOR1) + ",") >= 0 &&
                         cfg.indexOf("Role:1,") >= 0;

            flushBU04();

            if (!twrOk) {
                // Заводские или неверные настройки. Бурст: SETCFG → SETUWBMODE=1 → SAVE.
                // AT+SAVE встаёт в UART-буфер STM32 до того, как node_start() заблокирует.
                // После NVIC_SystemReset BU04 загружается с twr_pdoa_mode=1 → PDOA ветка.
                Serial.printf("# Первичная настройка PDOA: id=%u role=1 ch=%u rate=%u\n",
                              BU04_ID_ANCHOR1, BU04_CHANNEL, BU04_RATE);
                bu04.print("AT+SETCFG=" + String(BU04_ID_ANCHOR1) + ",1," +
                           String(BU04_CHANNEL) + "," + String(BU04_RATE) + "\r\n");
            } else {
                // ID/Role уже верные, только меняем режим — SETCFG не нужен,
                // node_start() не вызывается, INIT FAILED невозможен.
                Serial.println("# TWR конфиг корректен, переключаем в PDOA...");
            }

            // В обоих случаях: SETUWBMODE=1 (RAM) + SAVE (NVM + сброс)
            bu04.print("AT+SETUWBMODE=1\r\n");
            bu04.print("AT+SAVE\r\n");

            // SAVE запишет twr_pdoa_mode=1 в NVM и вызовет NVIC_SystemReset.
            // После перезагрузки BU04 идёт в PDOA-ветку, DW3000 стартует чисто.
            waitBU04ReadyForever("# BU04 не отвечает — выключите/включите питание ТОЛЬКО BU04 (не ESP32)");
            Serial.println("# BU04 готов в режиме PDOA");
        } else {
            Serial.println("# BU04 уже в режиме PDOA");
        }

        // Калибровочные поправки PDOA (D-02, D-03):
        // AT+PDOAOFF и AT+RNGOFF ДОЛЖНЫ отправляться ДО AT+USER_CMD=0.
        // После AT+USER_CMD=0 эти команды — no-op!
        {
            String offResp = sendAT(String(AT_PDOAOFF) + "=" + String(PDOA_OFFSET_DEG), 500);
            Serial.printf("# PDOAOFF=%d → %s\n", PDOA_OFFSET_DEG,
                          offResp.indexOf("OK") >= 0 ? "OK" : offResp.c_str());
            String rngResp = sendAT(String(AT_RNGOFF) + "=" + String(RANGE_OFFSET_CM), 500);
            Serial.printf("# RNGOFF=%d → %s\n", RANGE_OFFSET_CM,
                          rngResp.indexOf("OK") >= 0 ? "OK" : rngResp.c_str());
        }

        // Регистрация тега (D-04..D-08): ждём тег, добавляем, сохраняем в NVM
        getdlistAndRegister();

        // Устанавливаем JSON-вывод
        sendAT(AT_USER_CMD_JSON, 500);
        Serial.println("# Формат вывода: JSON");

        // Диагностический баннер при старте (D-09, FW-05)
        {
            // 1. Версия прошивки BU04
            String ver = sendAT(AT_GETVER, 600);
            ver.trim();
            Serial.printf("# BU04 версия: %s\n", ver.c_str());

            // 2. PDOA статус + зарегистрированный тег
            String cfg = sendAT(AT_GETCFG, 1000);
            cfg.trim();
            // AT+GETCFG в PDOA: "getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N ..."
            Serial.printf("# GETCFG: %s\n", cfg.c_str());

            // Извлекаем короткий и длинный адрес зарегистрированного тега из KList
            String klist = sendAT(AT_GETKLIST, 1000);
            klist.trim();
            {
                int la = klist.indexOf("LongAddr64:");
                int sa = klist.indexOf("ShortAddr:");
                String longA = (la >= 0) ? klist.substring(la + 11, la + 27) : "—";
                String shortA = "—";
                if (sa >= 0) {
                    String rest = klist.substring(sa + 10);
                    int sp = rest.indexOf(' ');
                    shortA = (sp >= 0) ? rest.substring(0, sp) : rest.substring(0, 4);
                }
                longA.trim();  shortA.trim();
                Serial.printf("# Тег: Short=%s  Long=%s\n",
                              shortA.c_str(), longA.c_str());
            }

            // 3. Калибровочные константы
            Serial.printf("# PDOAOFF=%d RNGOFF=%d\n",
                          PDOA_OFFSET_DEG, RANGE_OFFSET_CM);
        }
        Serial.println("# Начало стриминга PDOA...");
    }
#else
    configureBU04(BU04_ID_ANCHOR1, /*role=*/1);
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
    Serial.println("# ========================================");
    configureBU04(BU04_ID_TAG, /*role=*/0);
    {
        String ver = sendAT("AT+GETVER", 500);
        ver.trim();
        Serial.println("# " + ver);
        String cfg = sendAT(AT_GETCFG, 1000);
        cfg.trim();
        // AT+GETCFG в TWR: getcfg ID:X, Role:X, CH:X, Rate:X, Group:X
        Serial.println("# " + cfg);
    }
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
        // Pass-through: строки из Serial → BU04 (AT+ADDTAG, AT+GETDLIST и т.д.)
        // Буферизуем до '\n', затем отправляем целой строкой.
        {
            static String serialBuf;
            while (Serial.available()) {
                char c = (char)Serial.read();
                if (c == '\n' || c == '\r') {
                    serialBuf.trim();
                    if (serialBuf.length() > 0) {
                        bu04.println(serialBuf);
                        Serial.println("> " + serialBuf);
                        serialBuf = "";
                    }
                } else {
                    serialBuf += c;
                }
            }
        }

        // Читаем всё, что накопилось в буфере UART за период POLL_INTERVAL_MS.
        // Tag_Addr: строки → CSV. Остальное (ответы AT) → Serial с префиксом "< ".
        String lastLine;
        {
            String buf;
            uint32_t t0 = millis();
            while (millis() - t0 < (POLL_INTERVAL_MS - 10)) {
                while (bu04.available()) {
                    char c = (char)bu04.read();
                    if (c == '\n') {
                        buf.trim();
                        if (buf.length() > 0) {
                            if (buf.startsWith("Tag_Addr:")) {
                                lastLine = buf;
                            } else {
                                Serial.println("< " + buf);  // ответ AT-команды
                            }
                        }
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
