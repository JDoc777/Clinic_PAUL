#define UART_LINK Serial3

#include "UART.h"
#include "PinDefinitions.h"
#include "Motors.h"
#include "LCD.h"
#include "PCAServo.h"

namespace UART {

// -------------------- State --------------------
enum class HsState : uint8_t { WAIT_REQUEST, ESTABLISHED };
static HsState hs = HsState::WAIT_REQUEST;

enum class RxState : uint8_t { Find, Hdr, Pay, CrcCheck, EndWait };
static RxState  rxState = RxState::Find;
static uint8_t  hdr[3];
static uint16_t need = 0;
static uint8_t  buf[160];
static uint16_t idx = 0;
static uint8_t  crc_le_bytes[2];

static char     lineBuf[32];
static uint8_t  lineLen = 0;
static void resetLine() { lineLen = 0; lineBuf[0] = '\0'; }

// -------------------- CRC16-CCITT --------------------
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// -------------------- Framer (sender) ----------------
class Framer {
public:
  void sendTelemetry(const Payload& pay) {
    const uint16_t len = sizeof(Payload);

    uint8_t header[3];
    header[0] = TYPE_TELEM;
    header[1] = (uint8_t)(len & 0xFF);
    header[2] = (uint8_t)(len >> 8);

    uint16_t crc = crc16_ccitt(header, sizeof(header));
    crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pay), len, crc);

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

// -------------------- Command handling ----------------
static void applyCommand(uint8_t flags, int16_t m[4], uint8_t s[5],
                         const char* text, uint8_t text_len)
{
 Serial.print(flags);
  bool flag0_LED0       = flags & (1 << 0); // bit 0
  bool flag1_LED1       = flags & (1 << 1); // bit 1
  bool flag2_Relay      = flags & (1 << 2); // bit 2
  bool flag3_Handshake  = flags & (1 << 3); // bit 3
  bool flag4_reset      = flags & (1 << 4); // bit 4
  bool flag5_Reserved5  = flags & (1 << 5); // bit 5
  bool flag6_Reserved6  = flags & (1 << 6); // bit 6
  bool flag7_Reserved7  = flags & (1 << 7); // bit 7

  if (flag0_LED0) {
    digitalWrite(LED_Pi, HIGH);
    Serial.println("Pi has turned sucessfully");
  } else {
    digitalWrite(LED_Pi, LOW);
    Serial.println("Pi has not turned on sucessfully");
  }

  if (flag1_LED1) {
    digitalWrite(LED_Arduino, HIGH);
    Serial.println("Arduino has connected sucessfully");
  } else {
    digitalWrite(LED_Arduino, LOW);
    Serial.println("Arduino has not connected sucessfully");
  }

  if (flag2_Relay) {
    digitalWrite(Relay_motors, HIGH);
    Serial.println("Relay ON");
  } else {
    digitalWrite(Relay_motors, LOW);
    Serial.println("Relay OFF");
  }

//handshake if/else may change
  if (flag3_Handshake) {
    Serial.println("Handshake was successful!");
  }  else {
    Serial.println("Handshake was not successful!");
  }

  if (flag4_reset) {
    Serial.println("RESET FLAG TRUE — restarting...");
    delay(100);
    NVIC_SystemReset();
  }
  
  if (flag5_Reserved5) {
    Serial.println("This flag is set to TRUE");
  } else {
    Serial.println("This flag is set to FALSE");
  }

  if (flag6_Reserved6) {
    Serial.println("This flag is set to TRUE");
  } else {
    Serial.println("This flag is set to FALSE");
  }

  if (flag7_Reserved7) {
    Serial.println("This flag is set to TRUE");
  } else {
    Serial.println("This flag is set to FALSE");
  }

  int fl = m[0];
  int fr = m[1];
  int rl = m[2];
  int rr = m[3];

  // If your commands are already in PWM range [-255..255], this is enough:
  mecanumDrive(fl, fr, rl, rr);

//try this layout (setArms(s[0]Base, s[1]Shoulder ...... ))
  int Base = s[0];
  int Shoulder = s[1];
  int Elbow = s[2];
  int WristP = s[3];
  int Claw = s[4];

  setArms(Base, Shoulder, Elbow, WristP, Claw);

  // Serial.print("Servos: ");
  // for (int i = 0; i < 4; i++) { Serial.print(s[i]); Serial.print(' '); }
  // Serial.println();

  // Serial.print("Text: ");
  // for (uint8_t i = 0; i < text_len; i++) Serial.print(text[i]);
  // Serial.println();
  lcd_show_text(text, text_len);
}

// -------------------- Handshake -----------------------
static void handleHandshakeByte(uint8_t b) {
  if (b == '\n' || b == '\r') {
    lineBuf[lineLen] = '\0';
    if (lineLen > 0) {
      for (uint8_t i = 0; i < lineLen; ++i) {
        if (lineBuf[i] >= 'A' && lineBuf[i] <= 'Z') lineBuf[i] = lineBuf[i] - 'A' + 'a';
      }
      if (strcmp(lineBuf, HS_REQ) == 0) {
        UART_LINK.write((const uint8_t*)HS_ACK, strlen(HS_ACK));
        UART_LINK.write('\n');
        hs = HsState::ESTABLISHED;
      }
    }
    resetLine();
  } else {
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = (char)b;
    } else {
      resetLine();
    }
  }
}

// -------------------- Public API ----------------------
bool isEstablished() { return hs == HsState::ESTABLISHED; }

void resetHandshake() {
  hs = HsState::WAIT_REQUEST;
  rxState = RxState::Find;
  resetLine();
}

// RX-only service
void service() {
  while (UART_LINK.available()) {
    uint8_t b = (uint8_t)UART_LINK.read();

    if (!isEstablished()) {
      handleHandshakeByte(b);
      continue;
    }

    switch (rxState) {
      case RxState::Find:
        if (b == START_BYTE) { idx = 0; rxState = RxState::Hdr; }
        break;

      case RxState::Hdr:
        hdr[idx++] = b;
        if (idx == 3) {
          idx = 0;
          need = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
          if (need > sizeof(buf)) { rxState = RxState::Find; break; }
          rxState = (need == 0) ? RxState::CrcCheck : RxState::Pay;
        }
        break;

      case RxState::Pay:
        buf[idx++] = b;
        if (idx == need) { idx = 0; rxState = RxState::CrcCheck; }
        break;

      case RxState::CrcCheck:
        crc_le_bytes[idx++] = b;
        if (idx == 2) {
          idx = 0;
          uint16_t rx_crc = (uint16_t)crc_le_bytes[0] | ((uint16_t)crc_le_bytes[1] << 8);
          uint16_t calc = crc16_ccitt(hdr, 3);
          calc = crc16_ccitt(buf, need, calc);
          if (calc == rx_crc) rxState = RxState::EndWait;
          else rxState = RxState::Find;
        }
        break;

      case RxState::EndWait:
        if (b == END_BYTE) {
          if (hdr[0] == TYPE_CMD) {
            uint16_t off = 0;
            uint8_t flags = buf[off++];

            int16_t motors[4];
            for (int i = 0; i < 4; ++i) {
              motors[i] = (int16_t)((uint16_t)buf[off] | ((uint16_t)buf[off+1] << 8));
              off += 2;
            }

            uint8_t servos[4];
            for (int i = 0; i < 4; ++i) servos[i] = buf[off++];

            uint8_t lcd_len = (off < need) ? buf[off++] : 0;
            if (off + lcd_len > need) lcd_len = 0;
            const char* text = (const char*)(buf + off);

            applyCommand(flags, motors, servos, text, lcd_len);
          }
        }
        rxState = RxState::Find;
        break;
    }
  }
}

// TX-only
void sendTelemetry(const Payload& p) {
  if (isEstablished()) {
    framer.sendTelemetry(p);
  }
}

} // namespace UART
