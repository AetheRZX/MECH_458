#include "MotorControl.h"

// Define static members
volatile long MotorControl::m1_pos = 0;
volatile long MotorControl::m2_pos = 0;
long MotorControl::target_pos = 0;

long MotorControl::last_m1_pos = 0;
long MotorControl::last_m2_pos = 0;
float MotorControl::m1_rpm = 0;
float MotorControl::m2_rpm = 0;
unsigned long MotorControl::lastRpmCalcTime = 0;

int MotorControl::global_pwm1 = 0;
int MotorControl::global_pwm2 = 0;
float MotorControl::kp = KP_DEFAULT;
float MotorControl::ki = KI_DEFAULT;
float MotorControl::kd = KD_DEFAULT;
float MotorControl::integral_error = 0;
long MotorControl::last_sync_error = 0;
bool MotorControl::isSynced = false;

void MotorControl::init() {
    // Pin Setup
    pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

    // Initial Stop
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
}

void MotorControl::update() {
    // RPM Calculation Logic (moved from logging to here or keep separate? 
    // The original code calculated RPM inside the logging block. 
    // It's better to calculate it here if we want it updated regularly, 
    // but to match original logic, I'll add a method to update RPM or just let the logger call it?
    // Actually, the original code calculated RPM *only* when logging. 
    // Let's keep that behavior for now to avoid changing timing, 
    // but I'll expose a method to update RPM state that the Logger can call, 
    // or just do it in update() if we want better control.
    // For now, I'll leave RPM calc to be triggered by the Logger or a separate timer.
    // Wait, the original code did it inside the `if (millis() - lastLogTime > LOG_INTERVAL)` block.
    // I will implement a `calculateRPM` method that can be called.
    
    if (isSynced) {
        long error_m1 = target_pos - m1_pos;
        int base_speed = 0;

        // Deadband (Stop if close)
        if (abs(error_m1) < 10) {
            base_speed = 0;
            integral_error = 0; 
        } else {
            // Direction
            if (error_m1 > 0) digitalWrite(M1_DIR, LOW); // Check wiring if reversed!
            else digitalWrite(M1_DIR, HIGH); 
            
            // Speed Profile (Soft Start/Stop)
            if (abs(error_m1) < 200) base_speed = SPEED_SLOW; 
            else base_speed = SPEED_CRUISE; // Limiting max speed here
        }
        
        // Slave Logic (PID)
        long sync_error = m1_pos - m2_pos;
        integral_error += sync_error;
        integral_error = constrain(integral_error, -500, 500); // Anti-windup
        
        long derivative = sync_error - last_sync_error;
        last_sync_error = sync_error;
        
        float adjustment = (sync_error * kp) + (integral_error * ki) + (derivative * kd);
        
        global_pwm1 = base_speed;
        global_pwm2 = base_speed + adjustment;
        
        // Apply to Hardware
        if (base_speed == 0) {
           // Stop and Hold
           global_pwm1 = 0; global_pwm2 = 0; 
           digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
        } else {
           // Run
           digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
           int safe_pwm2 = constrain(global_pwm2, 0, 255);
           analogWrite(M1_PWM, global_pwm1);
           analogWrite(M2_PWM, safe_pwm2);
           // Sync Direction
           digitalWrite(M2_DIR, digitalRead(M1_DIR));
        }
    }
}

void MotorControl::manualJog(int motor, bool up) {
    isSynced = false; 
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    int pwm = SPEED_CRUISE; int duration = 100; 
    if (motor == 1) {
        digitalWrite(M1_BRAKE, HIGH); digitalWrite(M1_DIR, up ? LOW : HIGH);
        analogWrite(M1_PWM, pwm); delay(duration); analogWrite(M1_PWM, 0); digitalWrite(M1_BRAKE, LOW);
    } else {
        digitalWrite(M2_BRAKE, HIGH); digitalWrite(M2_DIR, up ? LOW : HIGH);
        analogWrite(M2_PWM, pwm); delay(duration); analogWrite(M2_PWM, 0); digitalWrite(M2_BRAKE, LOW);
    }
}

void MotorControl::setTarget(long target) {
    target_pos = target;
    isSynced = true; 
    digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
}

void MotorControl::setZero() {
    noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
    noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
    target_pos = 0; integral_error = 0; last_sync_error = 0; isSynced = true;
}

void MotorControl::emergencyHalt() {
    target_pos = m1_pos;
    isSynced = true;
}

// ISRs
void MotorControl::countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void MotorControl::countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }

// Getters
long MotorControl::getM1Pos() { return m1_pos; }
long MotorControl::getM2Pos() { return m2_pos; }
long MotorControl::getTargetPos() { return target_pos; }
int MotorControl::getPWM1() { return global_pwm1; }
int MotorControl::getPWM2() { return global_pwm2; }

// RPM needs to be calculated periodically
void MotorControl::updateRPM(unsigned long intervalMs) {
    float rpm_factor = (1000.0 / intervalMs) * 60.0 / ENCODER_PPR;
    m1_rpm = (m1_pos - last_m1_pos) * rpm_factor;
    m2_rpm = (m2_pos - last_m2_pos) * rpm_factor;
    last_m1_pos = m1_pos; 
    last_m2_pos = m2_pos;
}

float MotorControl::getM1RPM() { return m1_rpm; }
float MotorControl::getM2RPM() { return m2_rpm; }

