#include <Arduino.h>

//// CONFIGURATION ////
// Encoder Settings (AMT102-V: Set Switch 4 ON for 100 PPR)
const float ENCODER_PPR = 100.0; 

// Speed Limit for 0.1 m/s (approx 3000 RPM on T8 screw)
// Max PWM is 255 (~5400 RPM). 150 is roughly 0.1 m/s. (Try with jogging, if you think its too fast change cruise speed)
const int SPEED_CRUISE = 150; 
const int SPEED_SLOW   = 60;  // Landing speed

// Data Logging Rate
const int LOG_INTERVAL = 50; 

//// PINS ////
// Motor 1 (Left/Master)
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 13; // Interrupt Pin

// Motor 2 (Right/Slave)
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 2; // Interrupt Pin

//// VARIABLES ////
volatile long m1_pos = 0;
volatile long m2_pos = 0;
long target_pos = 0;

// RPM Calculation
long last_m1_pos = 0;
long last_m2_pos = 0;
float m1_rpm = 0;
float m2_rpm = 0;

// Global PWM (for logging)
int global_pwm1 = 0;
int global_pwm2 = 0;

// PID TUNING (Adjust these using the Plotter!)
float kp = 2.0;   // Strength (Try 0.5 to 5.0)
float ki = 0.05;  // Memory (Try 0.01 to 0.2)
float integral_error = 0;

//// STATE MACHINE ////
bool isSynced = false; 
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);

  // Pin Setup
  pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_A, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

  // Initial Stop
  digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
  analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);

  Serial.println("System Ready. Speed Limit: 0.1 m/s");
}

void loop() {
  
  //// 1. READ COMMANDS ////
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Manual Jogging
    if (input.equals("q")) { manualJog(1, true); }
    else if (input.equals("a")) { manualJog(1, false); }
    else if (input.equals("w")) { manualJog(2, true); }
    else if (input.equals("s")) { manualJog(2, false); }
    
    // Set Zero
    else if (input.equalsIgnoreCase("z")) {
      noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
      target_pos = 0; integral_error = 0; isSynced = true;
      Serial.println("MSG: Zero Set!"); 
    }
    
    // Emergency Halt (Sets target to current position)
    else if (input.equalsIgnoreCase("h")) {
      target_pos = m1_pos;
      isSynced = true;
      Serial.println("MSG: EMERGENCY HALT TRIGGERED");
    }

    // Move to Target
    else {
      long new_target = input.toInt();
      if (new_target == 0 && input != "0") { /* Invalid */ } 
      else {
         target_pos = new_target;
         isSynced = true; 
         digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
      }
    }
  }

  //// 2. SYNC LOOP ////
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
    
    float adjustment = (sync_error * kp) + (integral_error * ki);
    
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

  //// 3. LOGGING ////
  if (millis() - lastLogTime > LOG_INTERVAL) {
    // RPM Calc
    float rpm_factor = (1000.0 / LOG_INTERVAL) * 60.0 / ENCODER_PPR;
    m1_rpm = (m1_pos - last_m1_pos) * rpm_factor;
    m2_rpm = (m2_pos - last_m2_pos) * rpm_factor;
    last_m1_pos = m1_pos; last_m2_pos = m2_pos;

    // CSV Output
    Serial.print(millis()); Serial.print(",");
    Serial.print(target_pos); Serial.print(",");
    Serial.print(m1_pos); Serial.print(",");
    Serial.print(m2_pos); Serial.print(",");
    Serial.print(m1_rpm); Serial.print(",");
    Serial.print(m2_rpm); Serial.print(",");
    Serial.print(global_pwm1); Serial.print(",");
    Serial.println(global_pwm2);
    
    lastLogTime = millis();
  }
}

//// HELPERS ////
void manualJog(int motor, bool up) {
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

void countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }