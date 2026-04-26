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
// Источник: BU03/BU04 AT指令 V1.0.6, команда AT+SETCFG
//   AT+SETCFG=X1,X2,X3,1,X4
//   X1: ID устройства (0–10)
//   X2: роль (0 = тег/TAG, 1 = база/ANCHOR)
//   X3: канал (0 = CH9 / 7987.2 МГц,  1 = CH5 / 6489.5 МГц)
//   1 : скорость — всегда 1 (6.8 Мбит/с, только это значение поддерживается)
//   X4: группа (0–255). Базы нужно настраивать, теги — оставлять 0

#define BU04_CHANNEL      1    // CH5 (6489.5 МГц)
#define BU04_RATE         1    // 6.8 Мбит/с (единственный вариант)
#define BU04_GROUP        1    // группа для якорей (теги = 0)

// ID: тег и anchor1 ДОЛЖНЫ иметь одинаковый ID (требование протокола TWR)
#define BU04_ID_ANCHOR1   0
#define BU04_ID_ANCHOR2   1
#define BU04_ID_TAG       0    // = BU04_ID_ANCHOR1

// ----- Режим UWB -------------------------------------------
// Источник: AT+SETUWBMODE (раздел 5 PDF)
//   0 = TWR   — классическое двунаправленное измерение расстояния
//   1 = PDOA  — угол + расстояние одним якорем (2 антенны BU04)
//
// При PDOA якорь возвращает JSON-пакеты с дистанцией И углом к тегу.
// Для TWR используйте AT+DISTANCE; для PDOA — AT+USER_CMD=0 + AT+ADDTAG.
// ВНИМАНИЕ: после смены режима обязателен AT+SAVE.
//
// Текущий режим по умолчанию — PDOA (1 якорь + 1 тег, угол через
// фазовую разность). Для 3-модульной TWR-схемы соберите окружения
// `anchor1` + `anchor2` + `tag` (PDOA_MODE не определён).
// #define PDOA_MODE

// ----- Геометрия платформы (только для TWR) ----------------
// Точно измерьте расстояние между ANCHOR1 и ANCHOR2 (метры).
// Рекомендуется 20–50 см; большая база → точнее угол.
#define BASELINE_M        0.30f   // 30 см (по умолчанию)

// ----- ESP-NOW: ANCHOR2 → ANCHOR1 (только для TWR) ---------
// Порядок настройки:
//   1. Прошейте anchor1 и откройте Serial Monitor (115200).
//      Появится строка:  # MAC ANCHOR1: XX:XX:XX:XX:XX:XX
//   2. Скопируйте MAC в виде {0xXX,0xXX,0xXX,0xXX,0xXX,0xXX}
//   3. Вставьте ниже и прошейте anchor2.
#define ANCHOR1_MAC       {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // ← ЗАМЕНИТЬ!
#define ESPNOW_MAGIC      0xBB04   // маркер пакета (защита от чужих пакетов)

// ----- Тайминги --------------------------------------------
#define POLL_INTERVAL_MS    200    // период запроса AT+DISTANCE (мс)
#define ANCHOR2_STALE_MS   1000    // данные anchor2 «протухают» через N мс

// ----- Обработка данных ------------------------------------
#define MOVING_AVG_SAMPLES  5      // скользящее среднее (измерений)
#define MAX_VALID_DIST_M   50.0f   // максимально допустимое расстояние (м)

// ----- Строки AT-команд BU04 -------------------------------
// Источник: BU03/BU04 AT指令 V1.0.6
//
// Раздел 1: общие команды
#define AT_TEST        "AT"              // проверка связи → OK
#define AT_SAVE        "AT+SAVE"         // сохранить конфиг → OK + перезагрузка ~3 с
#define AT_GETVER      "AT+GETVER"       // версия ПО и железа → "getver software:V1.0.0,hardware:V1.0.0"
#define AT_RESTART     "AT+RESTART"      // перезагрузка
#define AT_RESTORE     "AT+RESTORE"      // сброс к заводским настройкам
// AT+GETCFG ответ в TWR-режиме:  "getcfg ID:X, Role:X, CH:X, Rate:X, Group:X"
// AT+GETCFG ответ в PDOA-режиме: "getcfg Dlist:N KList:N Net:XXXX AncID:N Rate:N Filter:N UserCmd:N pdoaOffset:N rngOffset:N"
#define AT_GETCFG      "AT+GETCFG"
// AT+SETCFG=ID,Role,CH,1,Group  — формируется в configureBU04()

// Раздел 2: измерения (TWR)
#define AT_DISTANCE    "AT+DISTANCE"     // дистанция → "distance: X.XXXXXX"
// ВНИМАНИЕ: AT+GETSENSOR поддерживает только BU03, BU04 — НЕ поддерживает!

// Раздел 3: параметры TWR
#define AT_GETDEV      "AT+GETDEV"       // получить коэффициенты (задержка, Калман и т.д.)
// AT+SETDEV=labelRate,antDelay,kalmanEn,Q,R,corrA,corrB,posEn,posDim

// Раздел 4: PDOA
// AT+SETUWBMODE=0 → TWR,  AT+SETUWBMODE=1 → PDOA  (требует AT+SAVE)
#define AT_GETUWBMODE  "AT+GETUWBMODE"
#define AT_SETUWBMODE_TWR   "AT+SETUWBMODE=0"
#define AT_SETUWBMODE_PDOA  "AT+SETUWBMODE=1"
// Для PDOA-режима (только якорь):
#define AT_DECA        "AT+DECA$"        // аутентификация PDOA
#define AT_GETDLIST    "AT+GETDLIST"     // список обнаруженных тегов
#define AT_GETKLIST    "AT+GETKLIST"     // список сопряжённых тегов
// AT+ADDTAG=LongAddr64,ShortAddr,MinRate,MaxRate,Mode — добавить тег в пару (MaxRate макс. = 64)
// AT+DELTAG=LongAddr64                                — удалить тег
#define AT_PDOAOFF     "AT+PDOAOFF"      // коррекция угла (якорь)
#define AT_RNGOFF      "AT+RNGOFF"       // коррекция расстояния (якорь)
#define AT_FILTER      "AT+FILTER"       // включить фильтрацию (якорь)
#define AT_USER_CMD_JSON "AT+USER_CMD=0" // формат вывода: JSON
#define AT_USER_CMD_HEX  "AT+USER_CMD=1" // формат вывода: HEX
// AT+PDOASETCFG=p1,p2,p3,p4,p5,p6,p7  — настройка PDOA
#define AT_PDOAGETCFG  "AT+PDOAGETCFG"  // получить параметры PDOA
