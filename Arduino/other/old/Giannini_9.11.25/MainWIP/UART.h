#pragma once
#include <Arduino.h>
#include <stdint.h>

// ===== UART module API =====
// Call UART::service(...) in loop(). If you pass a non-null Payload*, it will
// send telemetry at 'period_ms' after the handshake. You can also call
// UART::sendTelemetry(...) anytime after isEstablished() becomes true.
namespace UART {

// -------------------- Framing constants -----------------
static constexpr uint8_t START_BYTE = 0x7E;
static constexpr uint8_t END_BYTE   = 0x7F;
static constexpr uint8_t TYPE_TELEM = 0x31;    // Telemetry out to Pi
static constexpr uint8_t TYPE_CMD   = 0x21;    // Commands in from Pi

// -------------------- Handshake tokens ------------------
static constexpr const char* HS_REQ = "request";
static constexpr const char* HS_ACK = "ack";

// -------------------- Telemetry payload -----------------
#pragma pack(push, 1)
struct Payload {
  // Sonar
  int16_t  sonar_FR;
  int16_t  sonar_FL;
  int16_t  sonar_RR;
  int16_t  sonar_RL;

  // Encoders
  int32_t  enc_FR;
  int32_t  enc_FL;
  int32_t  enc_RR;
  int32_t  enc_RL;

  // DHT (Inside)
  uint8_t  dhti_hum_int;
  uint8_t  dhti_hum_dec;
  uint8_t  dhti_tmp_int;
  uint8_t  dhti_tmp_dec;

  // DHT (Outside)
  uint8_t  dhto_hum_int;
  uint8_t  dhto_hum_dec;
  uint8_t  dhto_tmp_int;
  uint8_t  dhto_tmp_dec;

  // Relays
  uint16_t relay_bits;

  // Gyro (deg)
  float    gyro_pitch;
  float    gyro_yaw;
  float    gyro_roll;

  // Accel (m/s^2 or g)
  float    accel_surge;
  float    accel_heave;
  float    accel_sway;
};
#pragma pack(pop)
static_assert(sizeof(Payload) == 58, "Payload must be 58 bytes");

// --- API ---
bool  isEstablished();                            // handshake complete?
void  resetHandshake();                           // force back to waiting
void  sendTelemetry(const Payload& p);            // immediate send (if HS ok)
void  service(const Payload* periodic_tx = nullptr,
              uint32_t period_ms = 100);          // call from loop()

} // namespace UART
