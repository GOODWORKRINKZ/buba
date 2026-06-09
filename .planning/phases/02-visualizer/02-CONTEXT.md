---
phase: 02-visualizer
status: Ready for planning
created: 2026-06-09
depends_on: 01-firmware-fixes
requirements:
  - VIZ-01
  - VIZ-02
  - VIZ-03
  - VIZ-04
  - VIZ-05
---

# Phase 2: Python Visualizer — Context

**Gathered:** 2026-06-09
**Status:** Ready for planning — decisions locked

<domain>
## Phase Boundary

Python-скрипт визуализатора: открывает Serial порт к якорю BU04, парсит PDOA-измерения, показывает живую полярную диаграмму (r = дистанция, θ = угол) и пишет все данные в CSV-лог.

**In scope:**
- `tools/visualizer.py` — основной скрипт визуализатора
- Парсинг двух форматов: JSON от стоковой STM32 прошивки и CSV от ESP32 прошивки
- Авто-определение порта якоря по VID:PID (0483:5740) или ручное указание через `--port`
- Полярная диаграмма: scatter-точки + trail (последние 200) + кольца допуска (±10 см на 1/2/3 м)
- CSV-логгер: все поля из JSON/CSV, файл `logs/YYYYMMDD_HHMMSS.csv`
- Индикатор потери сигнала: "LOST" если нет пакетов > 1 сек
- Авто-поиск всех BU04 на USB, определение якоря через `AT+DECA$`

**Out of scope:**
- Heatmap плотности (деferred — можно добавить позже как опцию)
- Окно для тега (тег не шлёт измерения в PDOA-режиме)
- Калибровка (Phase 3)
- Управление роботом (Phase 5)

</domain>

<decisions>
## Implementation Decisions

### Data Format Handling (VIZ-01, VIZ-02)
- **D-01:** Авто-определение формата по первой значащей строке: есть `{` → JSON-парсер (стоковая STM32 прошивка), есть `PDOA,` → CSV-парсер (ESP32 прошивка `anchor1_pdoa`). Формат запоминается на всю сессию.
- **D-02:** JSON-формат (от стокового STM32): парсить `{"TWR":{"a16":"...","R":seq,"T":ts,"D":range_cm,"P":angle_deg,"Xcm":...,"Ycm":...,"Angle":...}}`. Ключевые поля: `D` (расстояние в см), `P` (угол PDOA в °), `R` (порядковый номер), `T` (timestamp DW3000).
- **D-03:** CSV-формат (от ESP32): парсить `PDOA,addr,seq,range_m,angle_deg,x_m,y_m,ts_ms`. Пересчитывать range_m → range_cm для единообразия.
- **D-04:** Единый внутренний формат `Measurement` (dataclass/NamedTuple): `timestamp, range_cm, angle_deg, seq, raw_json`. Все поля кроме первых трёх — optional.

### Port Discovery (VIZ-01)
- **D-05:** При запуске без `--port`: сканировать `/dev/ttyACM*`, для каждого порта с VID:0483 PID:5740 (STM32 Virtual COM Port) отправить `AT+DECA$`. Порт ответивший `"Device":"PDOA Node"` → якорь.
- **D-06:** При запуске с `--port /dev/ttyXXX`: использовать указанный порт, авто-определение формата всё равно работает.
- **D-07:** `dtr=False` при открытии порта — предотвращает сброс ESP32-C3 (если якорь за ESP32). Для прямого STM32 USB — не критично, но и не мешает.
- **D-08:** Если найдено несколько PDOA Node — подключиться к первому, вывести предупреждение об остальных.

### Visualization (VIZ-02, VIZ-04, VIZ-05)
- **D-09:** Полярная диаграмма: `fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})`. Scatter-артист обновляется через `.set_offsets()`, не `ax.clear()`.
- **D-10:** Trail: последние 200 точек с затуханием alpha от 0.1 (старые) до 1.0 (новая). `deque(maxlen=200)` — совпадает с исходным планом.
- **D-11:** Кольца допуска: dashed-окружности на 1.0, 2.0, 3.0 м с полупрозрачной заливкой ±0.1 м (10 см). Цвета: зелёный/синий/оранжевый.
- **D-12:** "LOST" индикатор: красный текст по центру графика, если `time.time() - last_packet_time > 1.0`. Исчезает при следующем пакете.
- **D-13:** Всё на одном графике: scatter + trail + кольца + LOST. Heatmap — deferred, можно добавить как опцию `--heatmap`.

### CSV Logging (VIZ-03)
- **D-14:** CSV-файл: `logs/YYYYMMDD_HHMMSS.csv` (время старта сессии). Директория `logs/` создаётся если нет.
- **D-15:** Колонки: `timestamp,range_cm,angle_deg,seq,raw_json`. `raw_json` — полная строка от BU04 для отладки.
- **D-16:** `csv.DictWriter` с `flush()` каждые 10 строк (компромисс между производительностью и сохранностью данных).
- **D-17:** Header пишется при создании файла.

### Threading & Performance
- **D-18:** Два потока: daemon reader thread (читает serial, парсит, пишет в deque + csv) и main thread (matplotlib FuncAnimation). Блокировка через `threading.Lock` на deque.
- **D-19:** `FuncAnimation` интервал 100 мс (10 Hz) — достаточно для плавного отображения при ~4 Hz данных.
- **D-20:** Graceful shutdown: Ctrl+C в окне matplotlib → закрыть serial, дописать CSV, выйти с кодом 0.

### Multiple BU04 Handling
- **D-21:** Авто-поиск сканирует все STM32 порты. Определяет якорь по ответу на `AT+DECA$`. Подключается к первому найденному якорю.
- **D-22:** Если якорь не найден авто-поиском — вывести список найденных портов и их ответы, попросить указать `--port` явно.
- **D-23:** Тег BU04 (Role:0) игнорируется при авто-поиске — он не отвечает на `AT+DECA$` как "PDOA Node".

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Files
- `scripts/dual_monitor.py` — существующий двухпортовый монитор, паттерны pyserial + threading
- `scripts/bu04_terminal.py` — интерактивный AT-терминал, парсинг ответов BU04
- `src/main.cpp` — ESP32 прошивка, формат CSV (если используется)
- `include/config.h` — константы SERIAL_BAUD, PDOA_OFFSET_DEG, RANGE_OFFSET_CM

### Reference Data
- `.planning/phases/01-firmware-fixes/01-SERIAL-LOG.txt` — лог общения Ai_Thinker_PDOA утилиты с BU04 (формат JSON)
- `tools/Ai_Thinker_PDOA_V1_0_1.zip` — родная утилита (интерфейс, поведение)

### JSON Format (от стоковой STM32 прошивки)
```json
JS006A{"TWR": {
  "a16": "657B",      // короткий адрес тега (hex)
  "R": 0,              // порядковый номер измерения
  "T": 774068,         // timestamp DW3000
  "D": 121,            // дистанция (см)
  "P": 4,              // PDOA угол (градусы, signed)
  "Xcm": 15,           // X координата (см)
  "Ycm": 230,          // Y координата (см)
  "Angle": 4           // угол (градусы)
}}
```

### CSV Format (от ESP32 прошивки anchor1_pdoa)
```
PDOA,<addr_hex>,<seq>,<range_m>,<angle_deg>,<x_m>,<y_m>,<ts_ms>
PDOA,0001,42,1.35,4.0,1.20,0.62,123456
```

### Port Detection
- STM32 Virtual COM Port: VID 0x0483, PID 0x5740
- ESP32-C3 USB CDC: VID 0x303A, PID 0x1001 (или другие)
- `AT+DECA$` → ответ содержит `"Device":"PDOA Node"` → это якорь

### Build System
- Не требуется — чистый Python. Зависимости: `pyserial`, `matplotlib`, `numpy`.
- `requirements.txt` или `pip install pyserial matplotlib numpy`

</canonical_refs>

<deferred>
## Deferred Ideas

- Heatmap-режим (`--heatmap`) — цветовая карта плотности за N секунд
- Отдельное окно для тега (если тег начнёт слать данные)
- Выбор формата вручную (`--format json|csv`) — если авто-определение ошибается
- Выбор цветовой схемы (`--theme dark|light`)
- Запись сырых данных в `.pcap`-подобный формат для replay

</deferred>
