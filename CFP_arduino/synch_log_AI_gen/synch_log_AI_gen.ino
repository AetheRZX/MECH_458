#include <Arduino.h>

//// CONFIGURATION
// You must set this to the correct PPR from your calibration test
const float ENCODER_PPR = 100; 

// How often to send data to Python (in milliseconds)
const int LOG_INTERVAL = 50; 

//// PINS
// Motor 1 (Master / Left)
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 2; // Interrupt Pin

// Motor 2 (Slave / Right)
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 3; // Interrupt Pin

//// VARIABLES
volatile long m1_pos = 0;
volatile long m2_pos = 0;
long target_pos = 0;

// Variables for RPM Calculation
long last_m1_pos = 0;
long last_m2_pos = 0;
float m1_rpm = 0;
float m2_rpm = 0;

// Global PWM (for logging)
int global_pwm1 = 0;
int global_pwm2 = 0;

// PID Tuning
float kp = 2.0;  
float ki = 0.05; 
float integral_error = 0;

//// STATE MACHINE
bool isSynced = false; 
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);

  //// PIN SETUP
  pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
  
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

  //// INITIAL STATE
  digitalWrite(M1_BRAKE, LOW); 
  digitalWrite(M2_BRAKE, LOW);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  Serial.println("System Ready. Waiting for Python...");
}

void loop() {
  
  //// 1. READ SERIAL COMMANDS
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    if (input.equals("q")) { manualJog(1, true); }        
    else if (input.equals("a")) { manualJog(1, false); }  
    else if (input.equals("w")) { manualJog(2, true); }   
    else if (input.equals("s")) { manualJog(2, false); }  
    
    else if (input.equalsIgnoreCase("z")) {
      noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
      target_pos = 0; integral_error = 0; isSynced = true;    
      Serial.println("MSG: Zero Set!"); 
    }
    else {
      long new_target = input.toInt();
      if (new_target == 0 && input != "0") { /* Invalid */ } 
      else {
         target_pos = new_target;
         isSynced = true; 
         digitalWrite(M1_BRAKE, HIGH);
         digitalWrite(M2_BRAKE, HIGH);
      }
    }
  }

  //// 2. SYNCHRONIZATION LOOP
  if (isSynced) {
    long error_m1 = target_pos - m1_pos;
    int base_speed = 0;

    // Deadband
    if (abs(error_m1) < 10) {
        base_speed = 0;
        integral_error = 0; 
    } else {
        if (error_m1 > 0) digitalWrite(M1_DIR, LOW); 
        else digitalWrite(M1_DIR, HIGH); 
        
        if (abs(error_m1) < 200) base_speed = 80; 
        else base_speed = 150; 
    }
    
    // Slave Logic
    long sync_error = m1_pos - m2_pos;
    integral_error += sync_error;
    integral_error = constrain(integral_error, -500, 500);
    
    float adjustment = (sync_error * kp) + (integral_error * ki);
    
    global_pwm1 = base_speed;
    global_pwm2 = base_speed + adjustment;
    
    if (base_speed == 0) {
       global_pwm1 = 0; global_pwm2 = 0; 
       digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    } else {
       digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
       int safe_pwm2 = constrain(global_pwm2, 0, 255);
       analogWrite(M1_PWM, global_pwm1);
       analogWrite(M2_PWM, safe_pwm2);
       digitalWrite(M2_DIR, digitalRead(M1_DIR));
    }
  }

  //// 3. DATA LOGGING AND RPM CALCULATION
  if (millis() - lastLogTime > LOG_INTERVAL) {
    
    // Calculate RPM
    // RPM = (Delta Pulses / PPR) * (60000ms / Interval)
    long delta1 = m1_pos - last_m1_pos;
    long delta2 = m2_pos - last_m2_pos;
    
    // Constant 1200 comes from: (1000ms / 50ms) * 60 seconds
    float rpm_factor = (1000.0 / LOG_INTERVAL) * 60.0 / ENCODER_PPR;
    
    m1_rpm = delta1 * rpm_factor;
    m2_rpm = delta2 * rpm_factor;
    
    last_m1_pos = m1_pos;
    last_m2_pos = m2_pos;

    // Send CSV Data
    // Format: Time,Target,Pos1,Pos2,RPM1,RPM2,PWM1,PWM2
    Serial.print(millis());
    Serial.print(",");
    Serial.print(target_pos);
    Serial.print(",");
    Serial.print(m1_pos);
    Serial.print(",");
    Serial.print(m2_pos);
    Serial.print(",");
    Serial.print(m1_rpm);
    Serial.print(",");
    Serial.print(m2_rpm);
    Serial.print(",");
    Serial.print(global_pwm1);
    Serial.print(",");
    Serial.println(global_pwm2);
    
    lastLogTime = millis();
  }
}

//// FUNCTIONS
void manualJog(int motor, bool up) {
  isSynced = false; 
  digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
  int pwm = 100; int duration = 150; 
  if (motor == 1) {
    digitalWrite(M1_BRAKE, HIGH); digitalWrite(M1_DIR, up ? LOW : HIGH);
    analogWrite(M1_PWM, pwm); delay(duration); analogWrite(M1_PWM, 0); digitalWrite(M1_BRAKE, LOW);
  } else {
    digitalWrite(M2_BRAKE, HIGH); digitalWrite(M2_DIR, up ? LOW : HIGH);
    analogWrite(M2_PWM, pwm); delay(duration); analogWrite(M2_PWM, 0); digitalWrite(M2_BRAKE, LOW);
  }
}

//// INTERRUPTS
void countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }