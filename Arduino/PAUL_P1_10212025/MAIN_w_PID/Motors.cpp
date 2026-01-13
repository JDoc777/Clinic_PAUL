#include <Arduino.h>
#include "PinDefinitions.h"
#include "Motors.h"

#define MIN_ENABLE 255

// Encoder globals
extern volatile long posi1; // FR
extern volatile long posi2; // FL
extern volatile long posi3; // RL
extern volatile long posi4; // RR

// Wheel geometry
static constexpr float WHEEL_RADIUS = 0.04f;  // 80mm diameter => 40mm radius

// Encoder pulses per rev
static constexpr float PULSES_PER_REV = 111.25f;

// Deadzone and PWM limits
static constexpr int DEADZONE_PWM = 175;
static constexpr int MAX_PWM = 255;

// Feedforward gain
static constexpr float FEEDFORWARD_GAIN = 50.0f;

// Last PWM values applied
int currentSpeedFL = 0;
int currentSpeedFR = 0;
int currentSpeedRL = 0;
int currentSpeedRR = 0;

// Desired angular speeds (rad/s)
float desiredOmegaFL = 0;
float desiredOmegaFR = 0;
float desiredOmegaRL = 0;
float desiredOmegaRR = 0;

// PID controllers
PIDController pidFL, pidFR, pidRL, pidRR;

// Last encoder readings
static long lastFR = 0;
static long lastFL = 0;
static long lastRL = 0;
static long lastRR = 0;

// Convert encoder delta → angular velocity (rad/s)
static float countsToOmega(long dc, float dt) {
    if (dt <= 0) return 0.0f;

    float revs = (float)dc / PULSES_PER_REV;
    float rad = revs * 2.0f * PI;
    return rad / dt;
}

void initMotorPID(float kp, float ki, float kd) {
    pidFL = PIDController(kp, ki, kd);
    pidFR = PIDController(kp, ki, kd);
    pidRL = PIDController(kp, ki, kd);
    pidRR = PIDController(kp, ki, kd);

    noInterrupts();
    lastFR = posi1;
    lastFL = posi2;
    lastRL = posi3;
    lastRR = posi4;
    interrupts();
}

void stopMotors() {
    desiredOmegaFL = desiredOmegaFR = desiredOmegaRL = desiredOmegaRR = 0;

    pidFL.reset();
    pidFR.reset();
    pidRL.reset();
    pidRR.reset();

    applyPWM(0, 0, 0, 0);
}

// Pi sends linear speed (m/s × 100)
void mecanumDrive(int fl_cmd, int fr_cmd, int rl_cmd, int rr_cmd) {
    float fl_mps = fl_cmd / 100.0f;
    float fr_mps = fr_cmd / 100.0f;
    float rl_mps = rl_cmd / 100.0f;
    float rr_mps = rr_cmd / 100.0f;

    desiredOmegaFL = fl_mps / WHEEL_RADIUS;
    desiredOmegaFR = fr_mps / WHEEL_RADIUS;
    desiredOmegaRL = rl_mps / WHEEL_RADIUS;
    desiredOmegaRR = rr_mps / WHEEL_RADIUS;
}

void updateMotorPID(float dt) {
    long pFR, pFL, pRL, pRR;

    noInterrupts();
    pFR = posi1;
    pFL = posi2;
    pRL = posi3;
    pRR = posi4;
    interrupts();

    long dFR = pFR - lastFR;
    long dFL = pFL - lastFL;
    long dRL = pRL - lastRL;
    long dRR = pRR - lastRR;

    lastFR = pFR;
    lastFL = pFL;
    lastRL = pRL;
    lastRR = pRR;

    float omegaFR = countsToOmega(dFR, dt);
    float omegaFL = countsToOmega(dFL, dt);
    float omegaRL = countsToOmega(dRL, dt);
    float omegaRR = countsToOmega(dRR, dt);

    auto stopped = [](float w) {
        return fabs(w) < 0.05f; // rad/s threshold
    };

    float pwm_FL = stopped(desiredOmegaFL) ?
                   0.0f :
                   pidFL.compute(desiredOmegaFL, omegaFL, dt) +
                   desiredOmegaFL * FEEDFORWARD_GAIN;

    float pwm_FR = stopped(desiredOmegaFR) ?
                   0.0f :
                   pidFR.compute(desiredOmegaFR, omegaFR, dt) +
                   desiredOmegaFR * FEEDFORWARD_GAIN;

    float pwm_RL = stopped(desiredOmegaRL) ?
                   0.0f :
                   pidRL.compute(desiredOmegaRL, omegaRL, dt) +
                   desiredOmegaRL * FEEDFORWARD_GAIN;

    float pwm_RR = stopped(desiredOmegaRR) ?
                   0.0f :
                   pidRR.compute(desiredOmegaRR, omegaRR, dt) +
                   desiredOmegaRR * FEEDFORWARD_GAIN;

    // Deadzone
    auto dz = [](float x) {
        if (x > 0 && x < DEADZONE_PWM) return (float)DEADZONE_PWM;
        if (x < 0 && x > -DEADZONE_PWM) return (float)-DEADZONE_PWM;
        return x;
    };

    pwm_FL = dz(pwm_FL);
    pwm_FR = dz(pwm_FR);
    pwm_RL = dz(pwm_RL);
    pwm_RR = dz(pwm_RR);

    auto clamp = [](float x) {
        if (x > MAX_PWM) x = MAX_PWM;
        if (x < -MAX_PWM) x = -MAX_PWM;
        return (int)x;
    };

    currentSpeedFL = clamp(pwm_FL);
    currentSpeedFR = clamp(pwm_FR);
    currentSpeedRL = clamp(pwm_RL);
    currentSpeedRR = clamp(pwm_RR);

    applyPWM(currentSpeedFL, currentSpeedFR, currentSpeedRL, currentSpeedRR);
}

void applyPWM(int fl_pwm, int fr_pwm, int rl_pwm, int rr_pwm) {
    // FRONT LEFT
    if (fl_pwm > 0) { digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW); }
    else if (fl_pwm < 0) { digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH); fl_pwm = -fl_pwm; }
    else { digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW); }
    int pwm_FL = constrain(abs(fl_pwm), 0, MAX_PWM);
    if (pwm_FL == 0) pwm_FL = MIN_ENABLE;     // keep board awake
    analogWrite(ENA_FL, pwm_FL);

    // FRONT RIGHT
    if (fr_pwm > 0) { digitalWrite(IN4_FR, HIGH); digitalWrite(IN3_FR, LOW); }
    else if (fr_pwm < 0) { digitalWrite(IN4_FR, LOW); digitalWrite(IN3_FR, HIGH); fr_pwm = -fr_pwm; }
    else { digitalWrite(IN4_FR, LOW); digitalWrite(IN3_FR, LOW); }
    int pwm_FR = constrain(abs(fr_pwm), 0, MAX_PWM);
    if (pwm_FR == 0) pwm_FR = MIN_ENABLE;
    analogWrite(ENB_FR, pwm_FR);

    // REAR LEFT
    if (rl_pwm > 0) { digitalWrite(IN7_BL, HIGH); digitalWrite(IN8_BL, LOW); }
    else if (rl_pwm < 0) { digitalWrite(IN7_BL, LOW); digitalWrite(IN8_BL, HIGH); rl_pwm = -rl_pwm; }
    else { digitalWrite(IN7_BL, LOW); digitalWrite(IN8_BL, LOW); }
    int pwm_RL = constrain(abs(rl_pwm), 0, MAX_PWM);
    if (pwm_RL == 0) pwm_RL = MIN_ENABLE;
    analogWrite(ENB_BL, pwm_RL);

    // REAR RIGHT
    if (rr_pwm > 0) { digitalWrite(IN5_BR, HIGH); digitalWrite(IN6_BR, LOW); }
    else if (rr_pwm < 0) { digitalWrite(IN5_BR, LOW); digitalWrite(IN6_BR, HIGH); rr_pwm = -rr_pwm; }
    else { digitalWrite(IN5_BR, LOW); digitalWrite(IN6_BR, LOW); }
    int pwm_RR = constrain(abs(rr_pwm), 0, MAX_PWM);
    if (pwm_RR == 0) pwm_RR = MIN_ENABLE;
    analogWrite(ENA_BR, pwm_RR);
}
