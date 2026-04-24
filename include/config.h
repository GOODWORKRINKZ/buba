#pragma once

// ============================================================
//  config.h – BU04 (DW3000 + STM32F103) + ESP32-C3 SuperMini
//
//  BU04 управляется AT-командами по UART через STM32F103.
//  НЕ нужен SPI — DW3000 скрыт внутри модуля.
//
//  Подключение BU04 → ESP32-C3 SuperMini
//  ┌────────────────────────────────────────────────────────┐
//  │ BU04 пин     Назначение           ESP32-C3 пин        │
//  │ ──────────   ──────────────────   ───────────────────  │
//  │ 34 / 24 / 23 3V3 (питание)       3V3  (500 мА пик!)  │
//  │ 1 / 10       GND                 GND                  │
//  │ PA2 (пин 4)  USART2_TX (выход)   GPIO 3  ← RX        │
//  │ PA3 (пин 5)  USART2_RX (вход)    GPIO 2  → TX        │
//  └────────────────────────────────────────────────────────┘
//
//  Альтернативный UART (если USART2 не отвечает):
//  │ PA9  (пин 26) USART1_TX → GPIO 3                      │
//  │ PA10 (пин 27) USART1_RX ← GPIO 2                      │
//
//  Источник: BU04 规格书 V1.0.0, таблица 5 (управление пинами)
// ============================================================

// ----- UART к BU04 (USART2: PA2=TX, PA3=RX) ----------------
#define PIN_BU04_TX    2     // ESP32-C3 TX → BU04 PA3 (USART2_RX, пин 5)
#define PIN_BU04_RX    3     // ESP32-C3 RX ← BU04 PA2 (USART2_TX, пин 4)
#define BU04_BAUD      115200

// ----- USB Serial (телеметрия на ПК) -----------------------
#define SERIAL_BAUD    115200

// ----- Конфигурация BU04 -----------------------------------
//   role:    0 = тег (TAG, движется),  1 = база (ANCHOR, стоит)
//   channel: 0 = CH9 (7987.2 МГц),    1 = CH5  (6489.5 МГц)
//   rate:    0 = 850 кбит/с,           1 = 6.8  Мбит/с
#define BU04_CHANNEL      1    // CH5
#define BU04_RATE         1    // 6.8 Мбит/с

// ID: тег и anchor1 ДОЛЖНЫ иметь одинаковый ID (требование протокола)
#define BU04_ID_ANCHOR1   0
#define BU04_ID_ANCHOR2   1
#define BU04_ID_TAG       0    // = BU04_ID_ANCHOR1

// ----- Геометрия платформы ---------------------------------
// Точно измерьте расстояние между ANCHOR1 и ANCHOR2 (метры).
// Рекомендуется 20–50 см; большая база → точнее угол.
#define BASELINE_M        0.30f   // 30 см (по умолчанию)

// ----- ESP-NOW: ANCHOR2 → ANCHOR1 --------------------------
// Порядок настройки:
//   1. Прошейте anchor1 и откройте Serial Monitor (115200).
//      Появится строка:  # MAC ANCHOR1: XX:XX:XX:XX:XX:XX
//   2. Скопируйте MAC в виде {0xXX,0xXX,0xXX,0xXX,0xXX,0xXX}
//   3. Вставьте ниже и прошейте anchor2.
#define ANCHOR1_MAC       {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // ← ЗАМЕНИТЬ!
#define ESPNOW_MAGIC      0xBB04   // маркер пакета (защита от чужих пакетов)

// ----- Тайминги --------------------------------------------
#define POLL_INTERVAL_MS    200    // период запроса AT+DISTANCE (мс)
#define IMU_INTERVAL_MS     500    // период запроса AT+GETSENSOR (мс, только TAG)
#define ANCHOR2_STALE_MS   1000    // данные anchor2 «протухают» через N мс

// ----- Обработка данных ------------------------------------
#define MOVING_AVG_SAMPLES  5      // скользящее среднее (измерений)
#define MAX_VALID_DIST_M   50.0f   // максимально допустимое расстояние (м)

// ----- Строки AT-команд BU04 -------------------------------
// Источник: BU03/BU04 AT指令 v1.0.2 + опытные данные DipFlip/ultra-wideband-positioning
#define AT_GETCFG      "AT+GETCFG"
#define AT_GETVER      "AT+GETVER"
#define AT_DISTANCE    "AT+DISTANCE"
#define AT_GETSENSOR   "AT+GETSENSOR"   // акселерометр (только TTL-порт)
#define AT_SAVE        "AT+SAVE"        // сохранить конфиг → перезагрузка ~3 с
#define AT_RESTART     "AT+RESTART"
