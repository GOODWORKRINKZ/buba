#pragma once

// ============================================================
//  config.h – hardware pin map and protocol tuning
//
//  Wiring: BU04 UWB module  →  ESP32-C3 SuperMini
//  -------------------------------------------------------
//  BU04 pin   Signal    ESP32-C3 pin
//  --------   ------    ------------
//  VCC        3.3 V     3V3
//  GND        GND       GND
//  SCK        SPI_CLK   GPIO 4
//  MISO       SPI_MISO  GPIO 5
//  MOSI       SPI_MOSI  GPIO 6
//  NSS/CS     SPI_CS    GPIO 7
//  RST        RESET     GPIO 8
//  IRQ        INT       GPIO 9
// ============================================================

// ----- SPI / DW1000 pins -----------------------------------
#define PIN_SPI_SCK    4
#define PIN_SPI_MISO   5
#define PIN_SPI_MOSI   6
#define PIN_DW_CS      7    // chip-select (NSS)
#define PIN_DW_RST     8    // reset (active-low)
#define PIN_DW_IRQ     9    // interrupt (active-high)

// ----- Protocol tuning -------------------------------------
// How often the TAG initiates a new ranging round (ms)
#define RANGING_INTERVAL_MS   200

// Guard delay after transmit before switching to RX (µs)
#define TX_TO_RX_DELAY_US     1000

// RX timeout for any expected reply (ms, 0 = infinite)
#define RX_TIMEOUT_MS         200

// Speed of light in air (m/s), used for ToF → distance
#define SPEED_OF_LIGHT        299702547.0

// DW1000 timestamp resolution (seconds per DW1000 tick)
// 1 tick = ~15.65 ps  → 1 / (128 * 499.2e6)
#define DW1000_TIME_RES_S     (1.0 / (128.0 * 499.2e6))

// Maximum plausible distance (metres) – sanity-check filter
#define MAX_VALID_DISTANCE_M  300.0

// Number of consecutive measurements to smooth (moving avg)
#define MOVING_AVG_SAMPLES    5

// ----- DS-TWR message types --------------------------------
#define MSG_POLL              0x01   // TAG  → ANCHOR
#define MSG_POLL_ACK          0x02   // ANCHOR → TAG
#define MSG_FINAL             0x03   // TAG  → ANCHOR
#define MSG_RANGE_REPORT      0x04   // ANCHOR → TAG (distance)

// ----- Message frame layout --------------------------------
// All frames:
//   [0]  message type (uint8)
//   [1]  sequence number (uint8)
//   [2..] payload (type-specific)
//
// MSG_FINAL payload  [2..25]  three DW1000 40-bit timestamps:
//   t_poll_tx (5 bytes) | t_poll_ack_rx (5 bytes) | t_final_tx (5 bytes)
//
// MSG_RANGE_REPORT payload [2..5]:
//   distance in mm as uint32_t (little-endian)

#define FRAME_LEN_POLL          2
#define FRAME_LEN_POLL_ACK      2
#define FRAME_LEN_FINAL        17    // 2 header + 3×5 timestamps
#define FRAME_LEN_RANGE_REPORT  6    // 2 header + 4 bytes (mm)
