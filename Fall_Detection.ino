// Wifi connect only when fall is detected.

#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "secrets.h"

// WiFi credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* iftttWebhookURL = IFTTT_WEBHOOK_URL;

// Pin definitions
const int LED_PIN = 2;
const int BUZZER_PIN = 4;
const int SWITCH_PIN = 15;
const int MPU_INT_PIN = 19;

// Simple fall detection threshold
const float FALL_THRESHOLD = 1.5;   // Acceleration threshold in G

// Power saving interval settings
const int SENSOR_READ_INTERVAL_MS = 500;  // How often to read the accelerometer (500 = 0.5 seconds)
const int CPU_FREQUENCY_MHZ = 80;         // Lower CPU frequency to save power (240 is max)

// WiFi connection timeout
const int WIFI_CONNECTION_TIMEOUT_MS = 15000; // 15 seconds to connect to WiFi

Adafruit_MPU6050 mpu;

// State variables
bool fallDetected = false;
volatile bool motionDetected = false;
unsigned long lastSensorReadTime = 0;
unsigned long lastLedToggleTime = 0;
bool ledState = false;

// Interrupt handler for MPU motion detection
void IRAM_ATTR motionInterrupt() {
  motionDetected = true;
}

void configureMPUInterrupt() {
  // Set the motion interrupt pin as an input
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  
  // Set up MPU6050 for motion detection
  // Configure motion detection parameters
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(5);   // More sensitive to detect falls (1-255)
  mpu.setMotionDetectionDuration(1);    // Quick response to motion (1-255)
  mpu.setMotionInterrupt(true);
  
  // Install GPIO ISR service
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  
  // Attach the interrupt handler
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), motionInterrupt, RISING);
  
  Serial.println("MPU interrupt configured successfully");
}

bool connectToWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA); // Set station mode
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < WIFI_CONNECTION_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
    return true;
  } else {
    Serial.println("Failed to connect!");
    WiFi.disconnect(true);  // Disconnect and turn off WiFi
    WiFi.mode(WIFI_OFF);    // Turn off WiFi
    return false;
  }
}

bool sendAlertToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  HTTPClient http;
  http.begin(iftttWebhookURL);
  int httpResponseCode = http.GET();
  Serial.print("IFTTT response code: ");
  Serial.println(httpResponseCode);
  http.end();
  
  return (httpResponseCode == 200);
}

void disconnectWiFi() {
  Serial.println("Disconnecting WiFi to save power");
  WiFi.disconnect(true);  // Disconnect and turn off WiFi
  WiFi.mode(WIFI_OFF);    // Turn off WiFi
}

void setup() {
  // Set CPU frequency to save power
  setCpuFrequencyMhz(CPU_FREQUENCY_MHZ);
  
  Serial.begin(115200);
  delay(1000); // Give serial a moment to start
  Serial.println("Serial connection started");
  
  // Set pins as output/input
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  // LED test at startup - blink twice
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  delay(300);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);

  // Buzzer test at startup
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize Wire (I2C)
  Wire.begin();
  
  // Initialize MPU6050
  for (int attempt = 0; attempt < 5; attempt++) {
    if (mpu.begin()) {
      Serial.println("MPU6050 Found!");
      break;
    }
    Serial.println("Failed to find MPU6050 chip, retrying...");
    delay(500);
    if (attempt == 4) {
      Serial.println("Could not find MPU6050. Restarting device...");
      ESP.restart();
    }
  }
  
  // Set up MPU6050 with lower power settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Set up MPU interrupts
  configureMPUInterrupt();
  
  // Reset motion detected flag
  motionDetected = false;
  
  // Turn off WiFi initially
  WiFi.mode(WIFI_OFF);
  
  Serial.println("System ready - waiting for fall...");
  
  // Initialize timers
  lastSensorReadTime = millis();
  lastLedToggleTime = millis();
}

void loop() {
  // Track current time to minimize millis() calls
  unsigned long currentMillis = millis();
  
  // Only read sensors periodically to save power
  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS || motionDetected) {
    lastSensorReadTime = currentMillis;
    
    // Get sensor data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate total acceleration magnitude
    float accMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                             a.acceleration.y * a.acceleration.y + 
                             a.acceleration.z * a.acceleration.z);
    
    // Convert to G force
    float accGForce = accMagnitude / 9.8;
    
    // Switch detection
    bool switchPressed = (digitalRead(SWITCH_PIN) == LOW);
    
    // Check for motion interrupt
    if (motionDetected) {
      Serial.println("Motion detected by interrupt!");
      motionDetected = false; // Reset the flag
    }
    
    // Fall detection logic
    if ((accGForce >= FALL_THRESHOLD || switchPressed) && !fallDetected) {
      String reason = switchPressed ? "TEST BUTTON PRESSED" : "ACCELERATION THRESHOLD";
      Serial.println(reason + " - Fall detected! Acceleration: " + String(accGForce) + " G");
  
      fallDetected = true;
      
      // Turn on LED
      digitalWrite(LED_PIN, HIGH);
      
      // Start buzzer
      digitalWrite(BUZZER_PIN, HIGH);
  
      // Blink LED 5 times
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
      }
      
      // Keep LED on for visual indication
      digitalWrite(LED_PIN, HIGH);
      
      // === CONNECT TO WIFI ONLY WHEN FALL IS DETECTED ===
      Serial.println("Fall detected - connecting to WiFi to send alert...");
      bool connected = connectToWiFi();
      
      if (connected) {
        // Send alert
        bool alertSent = sendAlertToServer();
        Serial.println(alertSent ? "Alert sent successfully!" : "Failed to send alert!");
        
        // Disconnect WiFi to save power
        disconnectWiFi();
      } else {
        Serial.println("Could not connect to WiFi to send alert.");
      }
  
      // Turn off LED when done
      digitalWrite(LED_PIN, LOW);
      
      // Stop buzzer
      digitalWrite(BUZZER_PIN, LOW);
      
      Serial.println("Fall detection sequence complete. System reset.");
      
      // Reset fall detection
      fallDetected = false;
    }
  }
  
  // Short delay to prevent CPU from being 100% utilized
  delay(10);
}