// --- Interactive Calibration Sketch to Find Motor PPR ---

const int PIN_PWM = 5;
const int PIN_BRAKE = 4;
const int PIN_DIR = 8;
const int PIN_ENC_A = 2; // Interrupt pin

volatile long pulseCount = 0;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);

  // Attach interrupt to count pulses continuously
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), countPulse, RISING);

  // Ensure motor is stopped initially
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_BRAKE, LOW); // Brake engaged (Stop)
  
  Serial.println("--- Setup Complete ---");
}

void loop() {
  // 1. Wait for user input
  Serial.println("\nType any character and press ENTER to start the 1-second test...");
  
  // This while loop blocks the code until you send data via Serial Monitor
  while (Serial.available() == 0) {
    // Do nothing, just wait
  }
  
  // Clear the serial buffer (read the character you sent so we don't trigger twice)
  while(Serial.available() > 0) {
    Serial.read();
  }

  // 2. Prepare for Test
  Serial.println("Starting in 1 second...");
  delay(1000); // Short pause before movement
  
  // Reset Pulse Count
  noInterrupts();
  pulseCount = 0;
  interrupts();
  
  // 3. RUN MOTOR
  // Set direction
  digitalWrite(PIN_DIR, LOW); 
  // Release Brake
  digitalWrite(PIN_BRAKE, HIGH); 
  
  Serial.println("Running...");
  startTime = millis();
  
  // Set speed (adjust 100 to whatever speed allows you to count rotations easily)
  analogWrite(PIN_PWM, 100); 

  // 4. Wait exactly 1 second
  while(millis() - startTime < 1000) {
    // Just wait
  }
  
  // 5. STOP MOTOR
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_BRAKE, LOW); // Engage Brake
  
  // 6. Report Results
  Serial.println("--- Test Complete ---");
  Serial.print("Pulses counted: ");
  Serial.println(pulseCount);
  Serial.println("-----------------------------");
  Serial.println("Calculation: Pulses / Number of Rotations = PPR");
}

void countPulse() {
  pulseCount++;
}