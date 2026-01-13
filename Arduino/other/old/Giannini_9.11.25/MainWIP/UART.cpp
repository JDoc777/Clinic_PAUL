#include "UART.h"

// If you ever need to change the UART used for the link,
// define UART_LINK before including this file.
// Default: Arduino GIGA hardware UART on pins 14/15 (Serial3).
#ifndef UART_LINK
  #define UART_LINK Serial3
#endif

// -------------------- Handshake state -------------------
namespace UART {

enum class HsState : uint8_t { WAIT_REQUEST, ESTABLISHED };
static HsState hs = HsState::WAIT_REQUEST;

// -------------------- RX parser state -------------------
enum class RxState : uint8_t { Find, Hdr, Pay, CrcCheck, EndWait };
static RxState  rxState = RxState::Find;
static uint8_t  hdr[3];           // [0]=TYPE, [1]=LEN_LO, [2]=LEN_HI
static uint16_t need = 0;
static uint8_t  buf[160];
static uint16_t idx = 0;
static uint8_t  crc_le_bytes[2];

// -------------------- line buffer for ASCII handshake ---
static char     lineBuf[32];
static uint8_t  lineLen = 0;
static void resetLine() { lineLen = 0; lineBuf[0] = '\0'; }

// -------------------- CRC16-CCITT -----------------------
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// -------------------- Framer (sender) -------------------
class Framer {
public:
  void sendTelemetry(const Payload& pay) {
    const uint16_t len = sizeof(Payload);

    // Header = [TYPE][LEN(LE)]
    uint8_t header[3];
    header[0] = TYPE_TELEM;
    header[1] = (uint8_t)(len & 0xFF);
    header[2] = (uint8_t)(len >> 8);

    // CRC over [TYPE][LEN][PAYLOAD]
    uint16_t crc = crc16_ccitt(header, sizeof(header));
    crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pay), len, crc);

    // Emit: START, header, payload, CRC(LE), END
    const uint8_t start = START_BYTE;
    const uint8_t end   = END_BYTE;
    UART_LINK.write(&start, 1);
    UART_LINK.write(header, sizeof(header));
    UART_LINK.write(reinterpret_cast<const uint8_t*>(&pay), len);

    uint8_t crc_le[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
    UART_LINK.write(crc_le, 2);
    UART_LINK.write(&end, 1);
  }
};
static Framer framer;

// -------------------- Command handling ------------------
// Adjust this to actually drive motors/servos/relays in your project.
static void applyCommand(uint8_t flags, int16_t m[4], uint8_t s[4],
                         const char* text, uint8_t text_len)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, (flags & 0x01) ? HIGH : LOW);

  // Debug to USB (assumes Serial is available/initialized by your sketch)
  Serial.print("Flags: "); Serial.println(flags, BIN);

  Serial.print("Motors: ");
  for (int i = 0; i < 4; i++) { Serial.print(m[i]); Serial.print(' '); }
  Serial.println();

  Serial.print("Servos: ");
  for (int i = 0; i < 4; i++) { Serial.print(s[i]); Serial.print(' '); }
  Serial.println();

  Serial.print("Text: ");
  for (uint8_t i = 0; i < text_len; i++) Serial.print(text[i]);
  Serial.println();
}

// -------------------- Handshake byte handler ------------
static void handleHandshakeByte(uint8_t b) {
  // simple line reader (accepts \n or \r\n)
  if (b == '\n' || b == '\r') {
    lineBuf[lineLen] = '\0';
    if (lineLen > 0) {
      // to lowercase
      for (uint8_t i = 0; i < lineLen; ++i) {
        if (lineBuf[i] >= 'A' && lineBuf[i] <= 'Z') lineBuf[i] = lineBuf[i] - 'A' + 'a';
      }
      if (strcmp(lineBuf, HS_REQ) == 0) {
        UART_LINK.write((const uint8_t*)HS_ACK, strlen(HS_ACK));
        UART_LINK.write('\n');
        Serial.println("[HS] received 'request', sent 'ack'");
        hs = HsState::ESTABLISHED;
      } else {
        Serial.print("[HS] ignoring line: "); Serial.println(lineBuf);
      }
    }
    resetLine();
  } else {
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)b;
    } else {
      resetLine(); // overflow -> reset
    }
  }
}

// -------------------- Public API ------------------------
bool isEstablished() { return hs == HsState::ESTABLISHED; }

void resetHandshake() {
  hs = HsState::WAIT_REQUEST;
  rxState = RxState::Find;
  resetLine();
}

void sendTelemetry(const Payload& p) {
  if (isEstablished()) {
    framer.sendTelemetry(p);
  }
}

void service(const Payload* periodic_tx, uint32_t period_ms) {
  // ---------- Service inbound (handshake + frames) ----------
  while (UART_LINK.available()) {
    uint8_t b = (uint8_t)UART_LINK.read();

    // Gate on handshake first
    if (!isEstablished()) {
      handleHandshakeByte(b);
      continue;
    }

    // Framed parser
    switch (rxState) {
      case RxState::Find:
        if (b == START_BYTE) { idx = 0; rxState = RxState::Hdr; }
        break;

      case RxState::Hdr:
        hdr[idx++] = b;
        if (idx == 3) {
          idx = 0;
          need = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
          if (need > sizeof(buf)) {
            Serial.println("ERR: payload too big");
            rxState = RxState::Find;
            break;
          }
          rxState = (need == 0) ? RxState::CrcCheck : RxState::Pay;
        }
        break;

      case RxState::Pay:
        buf[idx++] = b;
        if (idx == need) { idx = 0; rxState = RxState::CrcCheck; }
        break;

      case RxState::CrcCheck:
        crc_le_bytes[idx++] = b;   // 2 bytes LE
        if (idx == 2) {
          idx = 0;
          uint16_t rx_crc = (uint16_t)crc_le_bytes[0] | ((uint16_t)crc_le_bytes[1] << 8);
          uint16_t calc = crc16_ccitt(hdr, 3);
          calc = crc16_ccitt(buf, need, calc);
          if (calc == rx_crc) rxState = RxState::EndWait;
          else { Serial.println("ERR: CRC mismatch"); rxState = RxState::Find; }
        }
        break;

      case RxState::EndWait:
        if (b == END_BYTE) {
          // Dispatch by TYPE
          uint8_t type = hdr[0];
          if (type == TYPE_CMD) {
            // Expected command layout:
            // flags (1) + 4*int16 motors + 4*uint8 servos + lcd_len (1) + text (lcd_len)
            Serial.print("RX CMD len="); Serial.println(need);
            if (need >= 1 + 4*2 + 4*1 + 1) {
              uint16_t off = 0;
              uint8_t flags = buf[off++];

              int16_t motors[4];
              for (int i = 0; i < 4; ++i) {
                motors[i] = (int16_t)((uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8));
                off += 2;
              }

              uint8_t servos[4];
              for (int i = 0; i < 4; ++i) servos[i] = buf[off++];

              uint8_t lcd_len = (off < need) ? buf[off++] : 0;
              if (off + lcd_len > need) lcd_len = 0;
              const char* text = (const char*)(buf + off);

              applyCommand(flags, motors, servos, text, lcd_len);
            } else {
              Serial.println("WARN: CMD payload too short");
            }
          } else {
            Serial.print("RX frame (unhandled type 0x");
            Serial.print(type, HEX);
            Serial.println(") ignored.");
          }
        } else {
          Serial.println("ERR: missing END");
        }
        rxState = RxState::Find;
        break;
    }
  }

  // ---------- Periodic telemetry send (after handshake) ----------
  static uint32_t last_tx = 0;
  const uint32_t now = millis();
  if (periodic_tx && isEstablished() && (now - last_tx) >= period_ms) {
    framer.sendTelemetry(*periodic_tx);
    last_tx = now;
  }
}

} // namespace UART
