#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <vector>
#include <cmath>

// Define pins
#define LED_PIN 2
#define BUZZER_PIN 14
#define BUTTON_PIN 15  // Emergency button

// Fall detection model parameters from your trained model
// IMPORTANT: Use actual trained values from fall_detection_model.h
const float MODEL_WEIGHTS[4] = { -0.09054448f, 0.00813667f, 0.04179285f, 0.00001560f };
const float MODEL_BIAS = -0.00065902f;

// Accelerometer
Adafruit_MPU6050 mpu;

// Window size for data collection (should match training)
const int WINDOW_SIZE = 40;

// Data storage
std::vector<float> x_values;
std::vector<float> y_values;
std::vector<float> z_values;

// Fall detection state
bool fallDetected = false;
unsigned long fallDetectedTime = 0;
const unsigned long FALL_ALERT_DURATION = 5000; // Alert for 5 seconds

// Function declarations
void extractFeatures(float features[4], const std::vector<float>& x, 
                     const std::vector<float>& y, const std::vector<float>& z);
bool isFall(const float *features);
void handleFallAlert();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Allow time for serial to connect
  
  Serial.println("\n\n---------------------------");
  Serial.println("ESP32 Fall Detection System");
  Serial.println("---------------------------");
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized successfully!");
  
  // Startup indicator
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 2000);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 2000);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);
  
  // Reserve capacity for data vectors
  x_values.reserve(WINDOW_SIZE);
  y_values.reserve(WINDOW_SIZE);
  z_values.reserve(WINDOW_SIZE);
  
  Serial.println("Fall detection system ready!");
  Serial.println("Monitoring accelerometer data...");
}

void loop() {
  // Check emergency button
  if (digitalRead(BUTTON_PIN) == LOW && !fallDetected) {
    Serial.println("Emergency button pressed!");
    fallDetected = true;
    fallDetectedTime = millis();
    handleFallAlert();
  }
  
  // Handle active fall alert
  if (fallDetected) {
    unsigned long currentTime = millis();
    if (currentTime - fallDetectedTime < FALL_ALERT_DURATION) {
      // Keep alert active - blink LED
      digitalWrite(LED_PIN, (currentTime / 500) % 2);
    } else {
      // End alert
      fallDetected = false;
      digitalWrite(LED_PIN, LOW);
      noTone(BUZZER_PIN);
      Serial.println("Alert ended. Monitoring for falls...");
    }
  }
  
  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Add samples to vectors
  x_values.push_back(a.acceleration.x);
  y_values.push_back(a.acceleration.y);
  z_values.push_back(a.acceleration.z);
  
  // Print raw data occasionally
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {  // Once per second
    Serial.print("Raw: X=");
    Serial.print(a.acceleration.x, 2);
    Serial.print(" Y=");
    Serial.print(a.acceleration.y, 2);
    Serial.print(" Z=");
    Serial.println(a.acceleration.z, 2);
    lastPrintTime = millis();
  }
  
  // Check if we have enough samples for a complete window
  if (x_values.size() >= WINDOW_SIZE) {
    // Extract features
    float features[4];
    extractFeatures(features, x_values, y_values, z_values);
    
    // Print features for debugging
    Serial.print("Features => Mean: ");
    Serial.print(features[0], 2);
    Serial.print(", Std: ");
    Serial.print(features[1], 2);
    Serial.print(", Max: ");
    Serial.print(features[2], 2);
    Serial.print(", Energy: ");
    Serial.println(features[3], 2);
    
    // Check for falls
    if (!fallDetected && isSignificantMovement(features) && isFall(features)) {
      Serial.println("FALL DETECTED!");
      fallDetected = true;
      fallDetectedTime = millis();
      handleFallAlert();
    }
    
    // Clear vectors for the next window
    // For a sliding window approach, you could keep a portion of the data
    x_values.clear();
    y_values.clear();
    z_values.clear();
  }
  
  // Control sampling rate (approximately 100Hz)
  delay(10);
}

// Extract features from window of accelerometer data
void extractFeatures(float features[4], const std::vector<float>& x, 
                     const std::vector<float>& y, const std::vector<float>& z) {
  // Calculate magnitude
  std::vector<float> mag(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    mag[i] = sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
  }
  
  // Mean magnitude (features[0])
  float sum = 0.0f;
  for (auto val : mag) sum += val;
  features[0] = sum / mag.size();
  
  // Standard deviation (features[1])
  float sum_sq_diff = 0.0f;
  for (auto val : mag) {
    float diff = val - features[0];
    sum_sq_diff += (diff * diff);
  }
  features[1] = sqrt(sum_sq_diff / mag.size());
  
  // Max magnitude (features[2])
  features[2] = 0;
  for (auto val : mag) {
    if (val > features[2]) features[2] = val;
  }
  
  // Energy (features[3])
  float energy = 0.0f;
  for (auto val : mag) energy += (val * val);
  features[3] = energy;
}

// Pre-check to filter out stationary or minimal movement
bool isSignificantMovement(const float *features) {
  // Thresholds for significant movement (adjust based on testing)
  if (features[1] < 0.5) {  // std_magnitude
    Serial.println("Movement too small - not a fall");
    return false;
  }
  
  if ((features[2] - features[0]) < 1.5) {  // max_mag - mean_mag
    Serial.println("No significant acceleration spike - not a fall");
    return false;
  }
  
  return true;
}

// Fall detection using logistic regression model
bool isFall(const float *features) {
  float score = MODEL_BIAS;
  for (int i = 0; i < 4; i++) {
    score += MODEL_WEIGHTS[i] * features[i];
  }
  
  Serial.print("Model score: ");
  Serial.println(score);
  
  // If score > 0, predict fall (probability > 0.5)
  return score > 0;
}

// Handle fall alert - activate LED, buzzer, etc.
void handleFallAlert() {
  // Visual alert - LED on
  digitalWrite(LED_PIN, HIGH);
  
  // Audible alert - buzzer
  tone(BUZZER_PIN, 2000);
  
  Serial.println("FALL ALERT ACTIVATED!");
  
}