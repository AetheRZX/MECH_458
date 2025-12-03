#include <Arduino.h>

const float ENCODER_PPR = 6.0;  // change zis

// PINS
// Motor 1 
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 2; // Int 0

// Motor 2 
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 3; // Int 1

// Variables
volatile long m1_pos = 0;
volatile long m2_pos = 0;
long target_pos = 0;

// PID, please tune kp and ki
float kp = 2.0;  
float ki = 0.05; 
float integral_error = 0;
float kd = 0.0;
long last_error = 0;

// State for zeroing
bool isSynced = false; // False = Manual Mode, True = Auto Move

// Forward declaration
void manualJog(int motor, bool up);
void countM1();
void countM2();
void runAutotune();

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
  
  // Encoders
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

  // STOP Motors Initially, remember to change as needed
  digitalWrite(M1_BRAKE, LOW); // engage brake
  digitalWrite(M2_BRAKE, LOW);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  Serial.println("Startup Ok");
  Serial.println("Controls:");
  Serial.println("  'q' / 'a' : Jog Motor 1 (Up / Down)");
  Serial.println("  'w' / 's' : Jog Motor 2 (Up / Down)");
  Serial.println("  'z'       : SET zero");
  Serial.println("  't'       : Run Autotune");
  Serial.println("  [number]  : Enter a value to Move platform");
  // Note that this is encoder pulses, so it probably will move as follow
  // input / PPR * pitch
}

void loop() {
  // Read serial from keyboard
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check for jog
    if (input.equals("q")) { manualJog(1, true); }
    else if (input.equals("a")) { manualJog(1, false); }
    else if (input.equals("w")) { manualJog(2, true); }
    else if (input.equals("s")) { manualJog(2, false); }
    else if (input.equals("t")) { runAutotune(); }
    
    // Check for Zero 
    else if (input.equalsIgnoreCase("z")) {
      noInterrupts();
      m1_pos = 0;
      m2_pos = 0;
      interrupts();
      target_pos = 0;
      isSynced = true; // Engage holding
      integral_error = 0;
      last_error = 0;
      Serial.println("Zero Set! Position is now 0.");
    }
    
    // Check for input
    else {
      long new_target = input.toInt();
      if (new_target == 0 && input != "0") {
         // Invalid input check
      } else {
         target_pos = new_target;
         isSynced = true;
         integral_error = 0;
         last_error = 0;
         Serial.print("Moving to: "); Serial.println(target_pos);
         // Release Brakes
         digitalWrite(M1_BRAKE, HIGH);
         digitalWrite(M2_BRAKE, HIGH);
      }
    }
  }

  // Synchronization loop 
  // THis only runs if we have set zero and entered a target
  if (isSynced) {
    
    // --- MASTER (M1) LOGIC ---
    long error_m1 = target_pos - m1_pos;
    int base_speed = 0;

    // Deadband (stop if close enough), change as needed. This is actually a good
    // metric to measure for our CFP so try to vary this (again this is in pulses)
    if (abs(error_m1) < 10) {
        base_speed = 0;
        integral_error = 0; // Reset integrator when stopped
    } else {
        // Direction, this needs to be change/checked
        if (error_m1 > 0) digitalWrite(M1_DIR, LOW); // UP
        else digitalWrite(M1_DIR, HIGH); // DOWN
        
        // Speed Profile
        if (abs(error_m1) < 200) base_speed = 80; // Slow landing, not sure if needed, set = base_speed if not
        else base_speed = 150; // Cruise speed, change as necessary
    }
    
    // SLAVE (M2) LOGIC
    // Calculate Sync Error 
    long sync_error = m1_pos - m2_pos;
    
    // PID Calculation
    integral_error += sync_error;
    // Anti-windup
    integral_error = constrain(integral_error, -500, 500);
    
    float derivative = sync_error - last_error;
    last_error = sync_error;
    float adjustment = (sync_error * kp) + (integral_error * ki) + (derivative * kd);
    
    // Calculate Final Speeds
    int pwm1 = base_speed;
    int pwm2 = base_speed + adjustment;
    
    // If Master stops, Slave must fight to stay at same position as Master
    if (base_speed == 0) {
       // If stopped, just use PID to hold position against M1
       pwm2 = adjustment; 
       // If pwm2 is negative, we need to reverse direction of M2...
       // But for simplicity in this specific setup:
       // If base is 0, we usually just Stop/Brake both.
       pwm1 = 0; 
       pwm2 = 0; 
       digitalWrite(M1_BRAKE, LOW); // Engage Brakes to hold
       digitalWrite(M2_BRAKE, LOW);
    } else {
       digitalWrite(M1_BRAKE, HIGH); // Release Brakes
       digitalWrite(M2_BRAKE, HIGH);
       
       // TO BE IMPLEMENTED
       // Handle Direction reversal for PID? 
       // (In this version: Assume we are moving generally in same direction)
       // If adjustment is huge negative, M2 might stall. 
       
       pwm2 = constrain(pwm2, 0, 255);
       analogWrite(M1_PWM, pwm1);
       analogWrite(M2_PWM, pwm2);
       
       // Ensure M2 Direction matches M1
       digitalWrite(M2_DIR, digitalRead(M1_DIR));
    }
  }
}

// MANUAL JOG FUNCTION 
void manualJog(int motor, bool up) {
  isSynced = false; // Disable auto-sync
  digitalWrite(M1_BRAKE, LOW);
  digitalWrite(M2_BRAKE, LOW);
  
  int pwm = 100; // Jog speed
  int duration = 200; // Move for 200ms then stop
  
  if (motor == 1) {
    digitalWrite(M1_BRAKE, HIGH);
    digitalWrite(M1_DIR, up ? LOW : HIGH);
    analogWrite(M1_PWM, pwm);
    delay(duration);
    analogWrite(M1_PWM, 0);
    digitalWrite(M1_BRAKE, LOW);
  } 
  else {
    digitalWrite(M2_BRAKE, HIGH);
    digitalWrite(M2_DIR, up ? LOW : HIGH);
    analogWrite(M2_PWM, pwm);
    delay(duration);
    analogWrite(M2_PWM, 0);
    digitalWrite(M2_BRAKE, LOW);
  }
  Serial.println("Jog done.");
}

// INTERRUPTS 
void countM1() {
  if (digitalRead(M1_DIR) == LOW) m1_pos++; 
  else m1_pos--; 
}

void countM2() {
  if (digitalRead(M2_DIR) == LOW) m2_pos++; 
  else m2_pos--; 
}

void runAutotune() {
  Serial.println("Starting Autotune (Relay Method)...");
  isSynced = false; 
  
  // 1. Lock Master (M1)
  digitalWrite(M1_BRAKE, LOW); // Engage Brake
  analogWrite(M1_PWM, 0);
  
  // 2. Prepare Slave (M2)
  digitalWrite(M2_BRAKE, HIGH); // Release Brake
  long start_pos = m1_pos; // Target is M1's current position
  
  // Tuning Parameters
  int relay_pwm = 100;      // Power for the relay step
  int hysteresis = 5;       // Noise buffer
  int cycles = 0;
  int max_cycles = 5;
  
  unsigned long t_last_cross = millis();
  unsigned long t_period_sum = 0;
  long amp_max = 0;
  long amp_min = 0;
  
  bool relay_state = false; // false = backward, true = forward
  
  Serial.println("Oscillating...");
  
  // Initial kick
  if (m2_pos < start_pos) {
      digitalWrite(M2_DIR, LOW); // Forward
      analogWrite(M2_PWM, relay_pwm);
      relay_state = true;
  } else {
      digitalWrite(M2_DIR, HIGH); // Backward
      analogWrite(M2_PWM, relay_pwm);
      relay_state = false;
  }
  
  unsigned long timeout = millis() + 10000; // 10s timeout
  
  while (cycles < max_cycles && millis() < timeout) {
      long error = start_pos - m2_pos;
      
      // Track Amplitude
      if (error > amp_max) amp_max = error;
      if (error < amp_min) amp_min = error;
      
      // Relay Switching
      if (relay_state && error < -hysteresis) {
          // Switch to Backward
          digitalWrite(M2_DIR, HIGH);
          analogWrite(M2_PWM, relay_pwm);
          relay_state = false;
          
          // Measure Period (on every full cycle, e.g., falling edge)
          unsigned long now = millis();
          unsigned long dt = now - t_last_cross;
          t_last_cross = now;
          
          if (cycles > 0) { // Skip first partial cycle
             t_period_sum += dt * 2; // Half cycle * 2 approx
          }
          cycles++;
      }
      else if (!relay_state && error > hysteresis) {
          // Switch to Forward
          digitalWrite(M2_DIR, LOW);
          analogWrite(M2_PWM, relay_pwm);
          relay_state = true;
      }
      
      delay(1); // Small loop delay
  }
  
  // Stop Motors
  analogWrite(M2_PWM, 0);
  digitalWrite(M2_BRAKE, LOW);
  
  if (cycles < max_cycles) {
      Serial.println("Autotune Failed: Timeout or no oscillation.");
      return;
  }
  
  // Calculate PID
  float avg_period_sec = (float)t_period_sum / (cycles - 1) / 1000.0; // Average period in seconds
  long peak_to_peak = amp_max - amp_min;
  float amplitude = peak_to_peak / 2.0;
  
  // Ziegler-Nichols (Classic PID)
  // Ku = 4 * d / (pi * a)
  // d = relay amplitude (PWM? No, effectively the force... but for position control, we approximate)
  // Actually, for position control, the "output" is PWM (0-255) and "input" is Error (counts).
  // This is tricky without a physical model. 
  // Standard Relay method: Ku = 4 * Output_Step / (PI * Input_Oscillation_Amp)
  
  float output_step = relay_pwm; 
  float Ku = (4.0 * output_step) / (3.14159 * amplitude);
  
  // PID Tuning Rules (Ziegler-Nichols)
  // Kp = 0.6 Ku
  // Ti = 0.5 Tu  -> Ki = Kp / Ti = 2 Kp / Tu
  // Td = 0.125 Tu -> Kd = Kp * Td = Kp * Tu / 8
  
  float new_kp = 0.6 * Ku;
  float new_ki = (2.0 * new_kp) / avg_period_sec;
  float new_kd = (new_kp * avg_period_sec) / 8.0;
  
  // Apply
  kp = new_kp;
  ki = new_ki;
  kd = new_kd;
  
  Serial.println("--- Autotune Complete ---");
  Serial.print("Period (s): "); Serial.println(avg_period_sec);
  Serial.print("Amplitude (cnts): "); Serial.println(amplitude);
  Serial.print("Ku: "); Serial.println(Ku);
  Serial.println("New PID Values:");
  Serial.print("Kp: "); Serial.println(kp, 4);
  Serial.print("Ki: "); Serial.println(ki, 4);
  Serial.print("Kd: "); Serial.println(kd, 4);
  
  // Reset for normal op
  integral_error = 0;
  last_error = 0;
}