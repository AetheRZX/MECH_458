#include <Arduino.h>

//// CONFIGURATION ////
// You must set this to the correct PPR from your calibration test
const float ENCODER_PPR = 6.0; 

// How often to send data to Python (in milliseconds)
const int LOG_INTERVAL = 50; 

//// PINS ////
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

//// VARIABLES ////
// Volatile is used because these change inside Interrupts
volatile long m1_pos = 0;
volatile long m2_pos = 0;

// The goal position where we want the platform to go
long target_pos = 0;

// Global variables to store current motor power (for logging)
int global_pwm1 = 0;
int global_pwm2 = 0;

// PID TUNING
// Kp: Reaction force. Higher = stiffer sync, but might oscillate.
// Ki: Memory. Higher = fixes small steady errors over time.
float kp = 2.0;  
float ki = 0.05; 
float integral_error = 0;

//// STATE MACHINE ////
// false = Manual Mode (Jogging), true = Auto Mode (PID Active)
bool isSynced = false; 

// Timer for logging data
unsigned long lastLogTime = 0;

void setup() {
  // Start Serial at high speed for smooth data transfer
  Serial.begin(115200);

  //// PIN SETUP ////
  pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
  
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  
  // Attach Interrupts to count pulses automatically
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

  //// INITIAL STATE ////
  // Motors stopped, Brakes engaged
  digitalWrite(M1_BRAKE, LOW); 
  digitalWrite(M2_BRAKE, LOW);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  Serial.println("System Ready. Waiting for Python or Commands...");
}

void loop() {
  
  //// 1. READ SERIAL COMMANDS ////
  // This listens for instructions from your PC (Python or Serial Monitor)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove spaces
    
    // Manual Jogging Commands
    if (input.equals("q")) { manualJog(1, true); }        // M1 UP
    else if (input.equals("a")) { manualJog(1, false); }  // M1 DOWN
    else if (input.equals("w")) { manualJog(2, true); }   // M2 UP
    else if (input.equals("s")) { manualJog(2, false); }  // M2 DOWN
    
    // Zero Command (Tell Arduino current height is 0)
    else if (input.equalsIgnoreCase("z")) {
      noInterrupts(); 
      m1_pos = 0; 
      m2_pos = 0; 
      interrupts();
      target_pos = 0;
      integral_error = 0; // Reset PID memory
      isSynced = true;    // Switch to auto-holding mode
      Serial.println("MSG: Zero Set!"); // "MSG:" prefix helps Python filter text
    }
    
    // Target Position Command (e.g., "2000")
    else {
      long new_target = input.toInt();
      // Simple check to ensure it's a number (0 is allowed if explicitly typed)
      if (new_target == 0 && input != "0") {
         // Invalid input, do nothing
      } else {
         target_pos = new_target;
         isSynced = true; // Enable Sync Loop
         // Release Brakes immediately
         digitalWrite(M1_BRAKE, HIGH);
         digitalWrite(M2_BRAKE, HIGH);
      }
    }
  }

  //// 2. SYNCHRONIZATION LOOP ////
  // This runs continuously to keep motors moving together
  if (isSynced) {
    
    // --- Master Motor Logic (M1) ---
    long error_m1 = target_pos - m1_pos;
    int base_speed = 0;

    // Deadband: If we are within 10 pulses, stop.
    if (abs(error_m1) < 10) {
        base_speed = 0;
        integral_error = 0; // Reset integrator so it doesn't wind up while stopped
    } else {
        // Set Direction
        if (error_m1 > 0) digitalWrite(M1_DIR, LOW); // UP (Check your wiring!)
        else digitalWrite(M1_DIR, HIGH); // DOWN
        
        // Speed Profile: Slow down when getting close
        if (abs(error_m1) < 200) base_speed = 80; // Landing speed
        else base_speed = 150; // Cruise speed
    }
    
    // --- Slave Motor Logic (M2) ---
    // Calculate difference between Master and Slave
    long sync_error = m1_pos - m2_pos;
    
    // PID: Integral Term (Accumulate error)
    integral_error += sync_error;
    // PID: Anti-windup (Limit the accumulation to prevent overshoot)
    integral_error = constrain(integral_error, -500, 500);
    
    // PID: Final Calculation
    float adjustment = (sync_error * kp) + (integral_error * ki);
    
    // Calculate final PWMs
    global_pwm1 = base_speed;
    global_pwm2 = base_speed + adjustment;
    
    // --- Apply to Hardware ---
    if (base_speed == 0) {
       // If Master stops, stop Slave and Engage Brakes
       global_pwm1 = 0; 
       global_pwm2 = 0; 
       digitalWrite(M1_BRAKE, LOW); 
       digitalWrite(M2_BRAKE, LOW);
    } else {
       // Release Brakes
       digitalWrite(M1_BRAKE, HIGH); 
       digitalWrite(M2_BRAKE, HIGH);
       
       // Ensure PWM is within 0-255 range
       int safe_pwm2 = constrain(global_pwm2, 0, 255);
       
       analogWrite(M1_PWM, global_pwm1);
       analogWrite(M2_PWM, safe_pwm2);
       
       // Simple Direction Sync: M2 copies M1's direction
       digitalWrite(M2_DIR, digitalRead(M1_DIR));
    }
  }

  //// 3. DATA LOGGING TO PYTHON ////
  // Send a CSV formatted line every 50ms
  if (millis() - lastLogTime > LOG_INTERVAL) {
    // Format: Timestamp,Target,M1,M2,PWM1,PWM2
    Serial.print(millis());
    Serial.print(",");
    Serial.print(target_pos);
    Serial.print(",");
    Serial.print(m1_pos);
    Serial.print(",");
    Serial.print(m2_pos);
    Serial.print(",");
    Serial.print(global_pwm1);
    Serial.print(",");
    Serial.println(global_pwm2); // println adds a new line at the end
    
    lastLogTime = millis();
  }
}

//// FUNCTIONS ////

// Manual Jog: Moves motor briefly then stops.
// Used for leveling the platform before setting Zero.
void manualJog(int motor, bool up) {
  isSynced = false; // Disable PID so it doesn't fight us
  
  // Release Brakes
  digitalWrite(M1_BRAKE, LOW); 
  digitalWrite(M2_BRAKE, LOW);
  
  int pwm = 100;      // Jog Speed
  int duration = 150; // Jog Time in ms
  
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
}

//// INTERRUPTS ////
// These run automatically whenever the encoder spins.
// We check the Direction Pin to know if we should Add or Subtract pulses.
void countM1() {
  if (digitalRead(M1_DIR) == LOW) m1_pos++; 
  else m1_pos--; 
}

void countM2() {
  if (digitalRead(M2_DIR) == LOW) m2_pos++; 
  else m2_pos--; 
}