#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Simple fall detection threshold
const float FALL_THRESHOLD = 1.5;   // Acceleration threshold in G

// LED pin
const int LED_PIN = 2;

// State variables
bool fallDetected = false;

void setup() {
  Serial.begin(115200);
  
  // Set up LED
  pinMode(LED_PIN, OUTPUT);
  
  // LED test at startup - blink twice
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  delay(300);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Fall Detection System Starting");
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
  // Set up MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("System ready - waiting for fall...");
}

void loop() {
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate total acceleration magnitude
  float accMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                            a.acceleration.y * a.acceleration.y + 
                            a.acceleration.z * a.acceleration.z);
  
  // Convert to G force
  float accGForce = accMagnitude / 9.8;
  
  // Print acceleration value
  Serial.print("Acceleration: ");
  Serial.print(accGForce);
  Serial.println(" G");
  
  // Simple fall detection
  if (accGForce >= FALL_THRESHOLD && !fallDetected) {
    Serial.println("FALL DETECTED! Acceleration: " + String(accGForce) + " G");
    fallDetected = true;
    
    // Turn on LED
    digitalWrite(LED_PIN, HIGH);
    
    // Blink LED 5 times
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
    
    // Turn off LED when done
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("Blinking complete. System reset.");
    
    // Reset fall detection
    fallDetected = false;
  }
  
  delay(100);
}