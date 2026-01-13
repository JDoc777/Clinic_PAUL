#pragma once
#include <stdint.h>

#define START_BYTE  0x7E
#define END_BYTE    0x7F
#define TYPE_CMD    0x21
#define TYPE_TELEM  0x31
#define HS_REQ      "request"
#define HS_ACK      "ack"

namespace UART {

#pragma pack(push,1)
struct Payload {
  int16_t sonar_F;
  int16_t sonar_B;
  int16_t sonar_L;
  int16_t sonar_R;

  int32_t enc_FR;
  int32_t enc_FL;
  int32_t enc_RL;
  int32_t enc_RR;

  // two DHTs = 8 bytes
  uint8_t dhti_h;
  uint8_t dhti_hdec;
  uint8_t dhti_t;
  uint8_t dhti_tdec;

  uint8_t dhto_h;
  uint8_t dhto_hdec;
  uint8_t dhto_t;
  uint8_t dhto_tdec;

  uint16_t flags;

  float gyro_roll;
  float gyro_pitch;
  float gyro_yaw;

  float accel_surge;
  float accel_sway;
  float accel_heave;
};
#pragma pack(pop)

void service();
void sendTelemetry(const Payload& p);
bool isEstablished();
void resetHandshake();

} // namespace UART
