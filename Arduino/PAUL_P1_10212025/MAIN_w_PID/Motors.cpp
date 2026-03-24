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

// If you want the same counts/rev used by your debug estimator from main,
// keep this at 223.0f like your original file.
// If your quadrature math really should be 111.25f, change it carefully
// everywhere together.
static constexpr float COUNTS_PER_REV_DEBUG = 223.0f;

// Deadzone and PWM limits
static constexpr int DEADZONE_PWM = 0; // 175
static constexpr int MAX_PWM = 255;

// Feedforward gain
float FL_GAIN = 10.7;
float FR_GAIN = 10.7;
float RL_GAIN = 11.33;
float RR_GAIN = 10.7;
static constexpr float FEEDFORWARD_GAIN = 1.00f; // 50

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

// Last encoder readings for PID control loop
static long lastFR = 0;
static long lastFL = 0;
static long lastRL = 0;
static long lastRR = 0;

// Last encoder readings for debug/raw/filter velocity estimate
static long prevFR_debug = 0;
static long prevFL_debug = 0;
static long prevRL_debug = 0;
static long prevRR_debug = 0;
static unsigned long prevTimeMicrosDebug = 0;

// Raw omega values for CSV/debug
float omegaFR_raw = 0.0f;
float omegaFL_raw = 0.0f;
float omegaRL_raw = 0.0f;
float omegaRR_raw = 0.0f;

// Filtered omega values for CSV/debug and PID feedback
float omegaFR_f = 0.0f;
float omegaFL_f = 0.0f;
float omegaRL_f = 0.0f;
float omegaRR_f = 0.0f;

static constexpr float TWO_PI_F = 6.28318530718f;

// Low-pass filter coefficients
static constexpr float LPF_A = 0.8f;
static constexpr float LPF_B = 0.2f;



// Convert encoder delta → angular velocity (rad/s)
static float countsToOmega(long dc, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float revs = (float)dc / PULSES_PER_REV;
    float rad = revs * 2.0f * PI;
    return rad / dt;
}

// Separate helper using the exact same formula from your original main file
static float countsToOmegaDebug(long dc, float dt) {
    if (dt <= 0.0f) return 0.0f;
    return (dc * TWO_PI_F) / (COUNTS_PER_REV_DEBUG * dt);
}

void updateMotorVelocityEstimate() {
    unsigned long now = micros();

    if (prevTimeMicrosDebug == 0) {
        prevTimeMicrosDebug = now;

        noInterrupts();
        prevFR_debug = posi1;
        prevFL_debug = posi2;
        prevRL_debug = posi3;
        prevRR_debug = posi4;
        interrupts();

        return;
    }

    float dt = (now - prevTimeMicrosDebug) * 1e-6f;
    if (dt <= 0.0f) return;

    long pFR, pFL, pRL, pRR;
    noInterrupts();
    pFR = posi1;
    pFL = posi2;
    pRL = posi3;
    pRR = posi4;
    interrupts();

    long dFR = pFR - prevFR_debug;
    long dFL = pFL - prevFL_debug;
    long dRL = pRL - prevRL_debug;
    long dRR = pRR - prevRR_debug;

    omegaFR_raw = countsToOmegaDebug(dFR, dt);
    omegaFL_raw = countsToOmegaDebug(dFL, dt);
    omegaRL_raw = countsToOmegaDebug(dRL, dt);
    omegaRR_raw = countsToOmegaDebug(dRR, dt);

    // Low-pass filter
    omegaFR_f = LPF_A * omegaFR_f + LPF_B * omegaFR_raw;
    omegaFL_f = LPF_A * omegaFL_f + LPF_B * omegaFL_raw;
    omegaRL_f = LPF_A * omegaRL_f + LPF_B * omegaRL_raw;
    omegaRR_f = LPF_A * omegaRR_f + LPF_B * omegaRR_raw;

    prevFR_debug = pFR;
    prevFL_debug = pFL;
    prevRL_debug = pRL;
    prevRR_debug = pRR;
    prevTimeMicrosDebug = now;
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

    prevFR_debug = posi1;
    prevFL_debug = posi2;
    prevRL_debug = posi3;
    prevRR_debug = posi4;
    interrupts();

    prevTimeMicrosDebug = micros();

    omegaFR_raw = 0.0f;
    omegaFL_raw = 0.0f;
    omegaRL_raw = 0.0f;
    omegaRR_raw = 0.0f;

    omegaFR_f = 0.0f;
    omegaFL_f = 0.0f;
    omegaRL_f = 0.0f;
    omegaRR_f = 0.0f;
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
// This used to be fl fr rl rr
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

    // Optional: if you want PID to use the exact same filtered values that get
    // printed to CSV, keep using omegaXX_f below.
    // That means updateMotorVelocityEstimate() must be called regularly.

    auto stopped = [](float w) {
        return fabs(w) < 0.05f; // rad/s threshold
    };

    float pwm_FL = stopped(desiredOmegaFL) ?
                   0.0f :
                   pidFL.compute(desiredOmegaFL, omegaFL_f, dt) +
                   desiredOmegaFL * FEEDFORWARD_GAIN * FL_GAIN;

    float pwm_FR = stopped(desiredOmegaFR) ?
                   0.0f :
                   pidFR.compute(desiredOmegaFR, omegaFR_f, dt) +
                   desiredOmegaFR * FEEDFORWARD_GAIN * FR_GAIN;

    float pwm_RL = stopped(desiredOmegaRL) ?
                   0.0f :
                   pidRL.compute(desiredOmegaRL, omegaRL_f, dt) +
                   desiredOmegaRL * FEEDFORWARD_GAIN * RL_GAIN;

    float pwm_RR = stopped(desiredOmegaRR) ?
                   0.0f :
                   pidRR.compute(desiredOmegaRR, omegaRR_f, dt) +
                   desiredOmegaRR * FEEDFORWARD_GAIN * RR_GAIN;

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
    // Serial.print("FL PWM:");
    // Serial.println(currentSpeedFL);
    // Serial.print("FR PWM:");
    // Serial.println(currentSpeedFR);
    // Serial.print("RL PWM:");
    // Serial.println(currentSpeedRL);
    // Serial.print("RR PWM:");
    // Serial.println(currentSpeedRR);


}

void applyPWM(int fl_pwm, int fr_pwm, int rl_pwm, int rr_pwm) {
    // FRONT LEFT
    if (fl_pwm > 0) {
        digitalWrite(IN1_FL, HIGH);
        digitalWrite(IN2_FL, LOW);
    } else if (fl_pwm < 0) {
        digitalWrite(IN1_FL, LOW);
        digitalWrite(IN2_FL, HIGH);
        fl_pwm = -fl_pwm;
    } else {
        digitalWrite(IN1_FL, HIGH);
        digitalWrite(IN2_FL, HIGH);
    }
    int pwm_FL = constrain(abs(fl_pwm), 0, MAX_PWM);
    //if (pwm_FL == 0) pwm_FL = MIN_ENABLE;     // keep board awake
    analogWrite(ENA_FL, pwm_FL);
    
    

    // FRONT RIGHT
    if (fr_pwm > 0) {
        digitalWrite(IN4_FR, HIGH);
        digitalWrite(IN3_FR, LOW);
    } else if (fr_pwm < 0) {
        digitalWrite(IN4_FR, LOW);
        digitalWrite(IN3_FR, HIGH);
        fr_pwm = -fr_pwm;
    } else {
        digitalWrite(IN4_FR, HIGH);
        digitalWrite(IN3_FR, HIGH);
    }
    int pwm_FR = constrain(abs(fr_pwm), 0, MAX_PWM);
    //if (pwm_FR == 0) pwm_FR = MIN_ENABLE;
    analogWrite(ENB_FR, pwm_FR);


    // REAR LEFT
    if (rl_pwm > 0) {
        digitalWrite(IN7_BL, HIGH);
        digitalWrite(IN8_BL, LOW);
    } else if (rl_pwm < 0) {
        digitalWrite(IN7_BL, LOW);
        digitalWrite(IN8_BL, HIGH);
        rl_pwm = -rl_pwm;
    } else {
        digitalWrite(IN7_BL, HIGH);
        digitalWrite(IN8_BL, HIGH);
    }
    int pwm_RL = constrain(abs(rl_pwm), 0, MAX_PWM);
    //if (pwm_RL == 0) pwm_RL = MIN_ENABLE;
    analogWrite(ENB_BL, pwm_RL);

    // REAR RIGHT
    if (rr_pwm > 0) {
        digitalWrite(IN5_BR, HIGH);
        digitalWrite(IN6_BR, LOW);
    } else if (rr_pwm < 0) {
        digitalWrite(IN5_BR, LOW);
        digitalWrite(IN6_BR, HIGH);
        rr_pwm = -rr_pwm;
    } else {
        digitalWrite(IN5_BR, HIGH);
        digitalWrite(IN6_BR, HIGH);
    }
    int pwm_RR = constrain(abs(rr_pwm), 0, MAX_PWM);
    //if (pwm_RR == 0) pwm_RR = MIN_ENABLE;
    analogWrite(ENA_BR, pwm_RR);
}