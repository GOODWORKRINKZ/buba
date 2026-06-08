/*
 * at_bridge.cpp – простой AT-командный мост
 *
 * Пробрасывает данные между USB Serial и BU04 UART:
 *   - Всё что введено в Serial Monitor → отправляется на BU04
 *   - Всё что пришло от BU04 → выводится в Serial Monitor
 *
 * Использование:
 *   1. Прошить: pio run -e at_bridge -t upload
 *   2. Открыть Serial Monitor (115200 baud)
 *   3. Вводить AT-команды, например:
 *      AT
 *      AT+GETCFG
 *      AT+SETCFG=0,0,1,1
 *      AT+SAVE
 *
 * Пины: GPIO2 = TX (→ BU04 RX), GPIO3 = RX (← BU04 TX)
 */

#ifdef ROLE_AT_BRIDGE

#include <Arduino.h>
#include <HardwareSerial.h>

#define BU04_TX_PIN  2
#define BU04_RX_PIN  3
#define BU04_BAUD    115200

static HardwareSerial bu04(1);

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n=== AT Bridge ===");
    Serial.printf("BU04: TX=GPIO%d RX=GPIO%d @ %d baud\n", BU04_TX_PIN, BU04_RX_PIN, BU04_BAUD);
    Serial.println("Вводите AT-команды (Enter для отправки)");
    Serial.println("Примеры: AT, AT+GETCFG, AT+SETCFG=0,0,1,1, AT+SAVE");
    Serial.println();

    bu04.begin(BU04_BAUD, SERIAL_8N1, BU04_RX_PIN, BU04_TX_PIN);
    delay(1000);
}

void loop() {
    // Serial → BU04
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.length() > 0) {
            Serial.print("> ");
            Serial.println(cmd);
            
            bu04.print(cmd);
            bu04.print("\r\n");
        }
    }
    
    // BU04 → Serial
    while (bu04.available()) {
        char c = bu04.read();
        Serial.write(c);
    }
}

#endif // ROLE_AT_BRIDGE
