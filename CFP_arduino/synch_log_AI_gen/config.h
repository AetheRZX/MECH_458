#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//// CONFIGURATION ////
// Encoder Settings (AMT102-V: Set Switch 4 ON for 100 PPR)
const float ENCODER_PPR = 100.0; 

// Speed Limit for 0.1 m/s (approx 3000 RPM on T8 screw)
// Max PWM is 255 (~5400 RPM). 150 is roughly 0.1 m/s.
const int SPEED_CRUISE = 150; 
const int SPEED_SLOW   = 60;  // Landing speed

// Data Logging Rate
const int LOG_INTERVAL = 50; 

//// PINS ////
// Motor 1 (Left/Master)
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 2; // Interrupt Pin

// Motor 2 (Right/Slave)
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 3; // Interrupt Pin

//// PID TUNING ////
const float KP_DEFAULT = 1.0;   // Strength (Try 0.5 to 5.0)
const float KI_DEFAULT = 0.05;  // Memory (Try 0.01 to 0.2)
const float KD_DEFAULT = 0.01;  // Damping (Try 0.0 to 1.0)

#endif
