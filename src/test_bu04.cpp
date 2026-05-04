/*
 * test_bu04.cpp – диагностика BU04 через UART
 *
 * Каждые 3 с отправляет "AT\r\n" на BU04, печатает сырой ответ (hex + ASCII).
 * Также работает как прозрачный pass-through:
 *   - всё что пришло от BU04 → Serial (с пометкой "<< ")
 *   - всё что набрано в Serial Monitor → BU04 (с пометкой ">> ")
 *
 * Пины: GPIO2 = TX (→ BU04 RX),  GPIO3 = RX (← BU04 TX)
 * Тест пинов: GPIO2 = TX (→ BU04 PA10=USART1_RX),  GPIO3 = RX (← BU04 PA9=USART1_TX)
 */

#ifdef ROLE_TEST

#include <Arduino.h>
#include <HardwareSerial.h>

#define BU04_TX_PIN  2
#define BU04_RX_PIN  3
#define BU04_BAUD    115200

static HardwareSerial bu04(1);

// Печатает строку байтов как hex + ASCII
static void hexdump(const String &s, const char *prefix) {
    Serial.print(prefix);
    Serial.print(" [");
    Serial.print(s.length());
    Serial.print("B] hex:");
    for (int i = 0; i < (int)s.length(); i++) {
        Serial.printf(" %02X", (uint8_t)s[i]);
    }
    Serial.print("  str:\"");
    for (int i = 0; i < (int)s.length(); i++) {
        char c = s[i];
        if (c >= 0x20 && c < 0x7F) Serial.print(c);
        else if (c == '\r')        Serial.print("\\r");
        else if (c == '\n')        Serial.print("\\n");
        else                       Serial.printf("\\x%02X", (uint8_t)c);
    }
    Serial.println("\"");
}

// Отправить команду и собрать ответ (до 2 с)
static String sendCmd(const String &cmd, uint32_t ms = 2000) {
    while (bu04.available()) bu04.read();   // flush
    bu04.print(cmd);
    bu04.print("\r\n");
    String resp;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        while (bu04.available()) resp += (char)bu04.read();
        if (resp.indexOf("OK")  >= 0) break;
        if (resp.indexOf("ERR") >= 0) break;   // BU04 шлёт "ERR", не "ERROR"
        delay(5);
    }
    return resp;
}

void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println("\n=== BU04 UART test ===");
    Serial.printf("TX=GPIO%d  RX=GPIO%d  baud=%d\n", BU04_TX_PIN, BU04_RX_PIN, BU04_BAUD);

    bu04.begin(BU04_BAUD, SERIAL_8N1, BU04_RX_PIN, BU04_TX_PIN);
    delay(2000);  // ждём boot BU04

    // ── Авто-последовательность настройки ────────────────────
    Serial.println("\n--- ШАГИ НАСТРОЙКИ ---");

    Serial.println("\n[1] AT (проверка связи)");
    hexdump(sendCmd("AT"), "<<");

    Serial.println("\n[2] AT+GETCFG (текущий конфиг)");
    hexdump(sendCmd("AT+GETCFG"), "<<");

    Serial.println("\n[3] AT+SETCFG=0,1,1,1 (id=0 role=anchor ch=5 rate=6.8M)");
    hexdump(sendCmd("AT+SETCFG=0,1,1,1"), "<<");

    Serial.println("\n[4] AT+GETCFG (проверяем что изменилось)");
    hexdump(sendCmd("AT+GETCFG"), "<<");

    Serial.println("\n[5] AT+SAVE (сохраняем)");
    hexdump(sendCmd("AT+SAVE", 500), "<<");   // после SAVE BU04 перезагружается

    Serial.println("\n--- SAVE отправлен, ждём 5 с на рестарт BU04 ---");
    delay(5000);

    Serial.println("\n[6] AT (BU04 после перезагрузки)");
    hexdump(sendCmd("AT"), "<<");

    Serial.println("\n[7] AT+GETCFG (сохранился ли конфиг после ресета?)");
    hexdump(sendCmd("AT+GETCFG"), "<<");

    Serial.println("\n--- Теперь сделайте POWER-CYCLE BU04 ---");
    Serial.println("--- После power-cycle введите AT+GETCFG вручную ---");
    Serial.println("--- Pass-through активен (каждые 3 с автопинг AT) ---");
}

static uint32_t lastAuto = 0;
static String   inputBuf;

void loop() {
    uint32_t now = millis();

    // ── Pass-through: Serial → BU04 ──────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuf.length() > 0) {
                Serial.print(">> ");
                Serial.println(inputBuf);
                String resp = sendCmd(inputBuf);
                if (resp.length() > 0)
                    hexdump(resp, "<<");
                else
                    Serial.println("<< (нет ответа за 1.5 с)");
                inputBuf = "";
            }
        } else {
            inputBuf += c;
        }
    }

    // ── Pass-through: BU04 → Serial (unsolicited) ────────────
    while (bu04.available()) {
        String line;
        uint32_t t0 = millis();
        while (millis() - t0 < 50) {
            if (bu04.available()) line += (char)bu04.read();
        }
        if (line.length()) hexdump(line, "<<");
    }

    // ── Авто-тест: "AT" каждые 3 с ───────────────────────────
    if (now - lastAuto >= 3000) {
        lastAuto = now;
        Serial.printf("\n[%lus] >> AT\n", now / 1000);
        String resp = sendCmd("AT");
        if (resp.length() > 0)
            hexdump(resp, "<<");
        else
            Serial.println("<< (нет ответа)");
    }
}

#endif // ROLE_TEST
