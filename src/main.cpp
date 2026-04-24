/*
 * main.cpp – BU04 Ai-Thinker UWB distance telemetry
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
