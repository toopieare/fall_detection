// Fall detector with improved sleep/wake cycle - interrupt & button wake only

#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "secrets.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// WiFi credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* iftttWebhookURL = IFTTT_WEBHOOK_URL;

// Pin definitions
const int LED_PIN = 2;
const int BUZZER_PIN = 4;
const int SWITCH_PIN = 15;
const int MPU_INT_PIN = 19;  // GPIO pin connected to INT pin of MPU6050

// Simple fall detection threshold
const float FALL_THRESHOLD = 1.5;   // Acceleration threshold in G

// Device state constants
enum DeviceState {
  STATE_SLEEPING,       // Light sleep mode, waiting for motion
  STATE_MONITORING,     // Actively monitoring for falls
  STATE_ALERT           // Fall detected, sending alert
};

// Timeout settings
const unsigned long INACTIVITY_TIMEOUT_MS = 10000;   // Go to sleep after 10 seconds of no motion
const unsigned long ACTIVE_MONITORING_MS = 5000;     // Monitor continuously for 5 seconds after motion
const unsigned long FORCE_SLEEP_TIMEOUT_MS = 30000;  // Force sleep after 30 seconds even with motion
const unsigned long DEBUG_INTERVAL_MS = 1000;        // How often to print debug info
const int CPU_FREQUENCY_MHZ = 80;                    // Lower CPU frequency to save power (240 is max)

// Motion detection settings
const float MOTION_THRESHOLD_G = 0.3;                // Threshold to consider motion present (in G)

// WiFi connection timeout
const int WIFI_CONNECTION_TIMEOUT_MS = 15000;        // 15 seconds to connect to WiFi

Adafruit_MPU6050 mpu;

// State variables
DeviceState currentState = STATE_MONITORING;
bool fallDetected = false;
volatile bool motionDetected = false;
unsigned long lastMotionTime = 0;
unsigned long stateStartTime = 0;
unsigned long lastDebugOutputTime = 0;
bool sleepEnabled = true;                            // Can be toggled via switch
bool justWokeUp = false;                             // Flag to track if we just woke up

// Interrupt handler for MPU motion detection
void IRAM_ATTR motionInterrupt() {
  motionDetected = true;
  // Brief flash to indicate interrupt triggered
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(20000);  // 20ms flash - shorter to reduce power usage
  digitalWrite(LED_PIN, LOW);
}

void configureMPUForSleep() {
  // Reset the MPU to ensure clean configuration
  mpu.reset();
  delay(100);  // Wait for reset to complete
  
  // Re-initialize with explicit I2C address
  Wire.begin();
  if (!mpu.begin(0x68)) {  // Default MPU6050 address
    Serial.println("Failed to find MPU6050 chip after reset!");
    // Try again
    delay(100);
    if (!mpu.begin(0x68)) {
      Serial.println("MPU initialization failed twice. Continuing anyway...");
    }
  }
  
  // Set MPU6050 for low power, high sensitivity to wake device
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // More sensitive range for sleep
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Set very sensitive motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);      // Very sensitive (1-255)
  mpu.setMotionDetectionDuration(1);       // Minimum duration to trigger
  mpu.setInterruptPinLatch(true);          // Latch interrupt pin until cleared
  mpu.setInterruptPinPolarity(false);      // Active LOW interrupt
  mpu.setMotionInterrupt(true);
  
  // Explicitly print interrupt settings
  uint8_t int_status = 0;
  Wire.beginTransmission(0x68);
  Wire.write(0x58);  // INT_ENABLE register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    int_status = Wire.read();
  }
  
  Serial.print("MPU Interrupt settings: 0x");
  Serial.println(int_status, HEX);
  Serial.println("MPU configured for sleep mode (maximum sensitivity)");
}

void configureMPUForMonitoring() {
  // Reset the MPU to ensure clean configuration
  mpu.reset();
  delay(100);  // Wait for reset to complete
  
  // Re-initialize
  Wire.begin();
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip after reset!");
    // Try again
    delay(100);
    if (!mpu.begin(0x68)) {
      Serial.println("MPU initialization failed twice. Continuing anyway...");
    }
  }
  
  // Set MPU6050 for more accurate fall detection
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Set motion detection for falls
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(5);      // Less sensitive
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptPinLatch(true);          // Latch interrupt pin until cleared
  mpu.setInterruptPinPolarity(false);      // Active LOW interrupt
  mpu.setMotionInterrupt(true);
  
  Serial.println("MPU configured for monitoring mode");
}

void setupMPUInterrupts() {
  // Set the motion interrupt pin as an input with pull-up
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  
  // Install GPIO ISR service if not already installed
  esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.println("Failed to install GPIO ISR service");
  }
  
  // Detach any existing interrupt first
  detachInterrupt(digitalPinToInterrupt(MPU_INT_PIN));
  
  // Attach the interrupt handler - interrupt is ACTIVE LOW
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), motionInterrupt, FALLING);
  
  Serial.println("MPU interrupt configured on pin " + String(MPU_INT_PIN));
}

void configureWakeupSources() {
  // First, disable all wake sources to start fresh
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  
  // Add switch pin as a wake source (active LOW)
  gpio_wakeup_enable((gpio_num_t)SWITCH_PIN, GPIO_INTR_LOW_LEVEL);
  
  // Configure MPU interrupt pin as wake source - just use LOW level
  gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_LOW_LEVEL);
  
  // Enable GPIO as wake sources
  esp_sleep_enable_gpio_wakeup();
  
  Serial.println("Wake sources configured: MPU interrupt on pin " + String(MPU_INT_PIN) + " and Button on pin " + String(SWITCH_PIN));
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

// Function to test if MPU interrupt is working
void testMPUInterrupt() {
  Serial.println("Testing MPU interrupt...");
  
  // Reset motion detection flag
  motionDetected = false;
  
  // Wait for interrupt or timeout
  unsigned long startTime = millis();
  Serial.println("Move the device now to generate motion interrupt...");
  
  while (!motionDetected && millis() - startTime < 5000) {
    Serial.print(".");
    delay(500);
  }
  
  if (motionDetected) {
    Serial.println("\nMotion interrupt successfully detected!");
  } else {
    Serial.println("\nNo motion interrupt detected after 5 seconds.");
    Serial.println("There may be an issue with the interrupt connection.");
  }
}

void clearMPUInterrupts() {
  // Read the INT_STATUS register to clear the interrupt
  Wire.beginTransmission(0x68);
  Wire.write(0x3A);  // INT_STATUS register
  Wire.endTransmission(false);
  
  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    uint8_t int_status = Wire.read();
    Serial.print("Cleared MPU interrupt status: 0x");
    Serial.println(int_status, HEX);
  }
  
  // Also clear our flag
  motionDetected = false;
}

// Function to display LED pattern for different events
void displayLEDPattern(int pattern) {
  switch (pattern) {
    case 1: // Sleep indication - 3 quick flashes
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(50);
      }
      break;
      
    case 2: // Wake up pattern - one long flash
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      break;
      
    case 3: // Button wake indication - two long flashes
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
      }
      break;
      
    case 4: // Fall detected - 5 quick flashes
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      break;
  }
}

void enterLightSleep() {
  if (!sleepEnabled) {
    Serial.println("Sleep mode disabled. Staying awake.");
    return;
  }
  
  Serial.println("\n----- Entering light sleep mode -----");
  
  // Turn off LED if it's on
  digitalWrite(LED_PIN, LOW);
  
  // Make sure buzzer is off
  digitalWrite(BUZZER_PIN, LOW);
  
  // Configure MPU for sleep (more sensitive to movement)
  configureMPUForSleep();
  
  // Run a quick interrupt test
  testMPUInterrupt();
  
  // Clear any pending interrupts before sleep
  clearMPUInterrupts();
  
  // Set up all possible wake sources
  configureWakeupSources();
  
  // Update state before sleeping
  currentState = STATE_SLEEPING;
  
  // Indicate we're going to sleep with LED pattern
  displayLEDPattern(1);
  
  // IMPORTANT: Wait a moment to make sure all operations complete
  delay(100);
  
  // Flush serial before sleeping
  Serial.flush();
  
  // Enter light sleep mode
  Serial.println("Sleeping now! (Device will stay asleep until motion or button press)");
  delay(50);  // Give serial time to send
  
  // Set the flag before sleep to detect when we wake up
  justWokeUp = true;
  
  // Enter light sleep
  esp_light_sleep_start();
  
  // Code resumes here after waking up
  Serial.println("\n\n----- DEVICE WOKE UP -----");
  
  // Display wake up pattern immediately for visual feedback
  displayLEDPattern(2);
  
  // Give a little time for system to stabilize
  delay(100);
  
  // Check wake reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.print("Woke up from sleep! Reason: ");
  
  bool buttonWake = false;
  
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_GPIO:
      // Check if it was the button that woke us
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.println("Button press");
        buttonWake = true;
        // Show special pattern for button wake
        displayLEDPattern(3);
      } else {
        Serial.println("MPU motion interrupt");
      }
      break;
    default:
      Serial.print("Other (");
      Serial.print(wakeup_reason);
      Serial.println(")");
      break;
  }
  
  // Configure MPU for active monitoring
  configureMPUForMonitoring();
  setupMPUInterrupts();
  
  // Update state after waking
  currentState = STATE_MONITORING;
  stateStartTime = millis();
  lastMotionTime = millis();
  lastDebugOutputTime = millis();
  
  Serial.println("----- Resumed monitoring mode -----\n");
}

void handleMonitoringState() {
  unsigned long currentTime = millis();
  
  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate total acceleration magnitude
  float accMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                           a.acceleration.y * a.acceleration.y + 
                           a.acceleration.z * a.acceleration.z);
  
  // Convert to G force
  float accGForce = accMagnitude / 9.8;
  
  // Check for manual button press
  bool switchPressed = (digitalRead(SWITCH_PIN) == LOW);
  
  // Debug output (limit frequency to not flood serial)
  if (currentTime - lastDebugOutputTime > DEBUG_INTERVAL_MS) {
    lastDebugOutputTime = currentTime;
    
    // Calculate time remaining before sleep
    unsigned long timeInState = currentTime - stateStartTime;
    unsigned long timeSinceMotion = currentTime - lastMotionTime;
    unsigned long sleepIn = 0;
    
    if (timeSinceMotion < INACTIVITY_TIMEOUT_MS) {
      sleepIn = INACTIVITY_TIMEOUT_MS - timeSinceMotion;
    }
    
    unsigned long forceTimeRemaining = 0;
    if (timeInState < FORCE_SLEEP_TIMEOUT_MS) {
      forceTimeRemaining = FORCE_SLEEP_TIMEOUT_MS - timeInState;
    }
    
    Serial.print("Monitoring - Acc: ");
    Serial.print(accGForce);
    Serial.print("G, Int pin: ");
    Serial.print(digitalRead(MPU_INT_PIN));
    Serial.print(", Time in state: ");
    Serial.print(timeInState / 1000.0);
    Serial.print("s, Sleep in: ");
    Serial.print(sleepIn / 1000.0);
    Serial.print("s, Force sleep in: ");
    Serial.print(forceTimeRemaining / 1000.0);
    Serial.println("s");
  }
  
  // Check if acceleration indicates motion
  bool currentMotion = (accGForce >= MOTION_THRESHOLD_G) || motionDetected;
  
  // If motion detected, update the last motion time
  if (currentMotion) {
    if (motionDetected) {
      Serial.println("Motion interrupt triggered!");
      motionDetected = false;  // Reset the flag
    }
    
    if (currentTime - lastMotionTime > 500) {  // Don't spam motion detected messages
      Serial.print("Motion detected: ");
      Serial.print(accGForce);
      Serial.println("G");
      lastMotionTime = currentTime;
    }
  }
  
  // Fall detection logic
  if ((accGForce >= FALL_THRESHOLD || switchPressed) && !fallDetected) {
    String reason = switchPressed ? "TEST BUTTON PRESSED" : "ACCELERATION THRESHOLD";
    Serial.println(reason + " - Fall detected! Acceleration: " + String(accGForce) + " G");
    
    fallDetected = true;
    currentState = STATE_ALERT;
    stateStartTime = currentTime;
    
    // Will be handled in the main loop state machine
    return;
  }
  
  // Check if we should go back to sleep (no motion for timeout period)
  bool shouldSleep = false;
  
  // Check inactivity timeout
  if (currentTime - lastMotionTime > INACTIVITY_TIMEOUT_MS && 
      currentTime - stateStartTime > ACTIVE_MONITORING_MS) {
    Serial.println("No motion detected for timeout period, going to sleep");
    shouldSleep = true;
  }
  
  // Force sleep after maximum active time regardless of motion
  if (currentTime - stateStartTime > FORCE_SLEEP_TIMEOUT_MS) {
    Serial.println("Force sleep timeout reached, going to sleep regardless of motion");
    shouldSleep = true;
  }
  
  // Enter sleep if conditions are met
  if (shouldSleep) {
    enterLightSleep();
  }
}

void handleAlertState() {
  // Turn on LED
  digitalWrite(LED_PIN, HIGH);
  
  // Start buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  
  // Display fall detected pattern
  displayLEDPattern(4);
  
  // Keep LED on for visual indication
  digitalWrite(LED_PIN, HIGH);
  
  // Connect to WiFi and send alert
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
  
  Serial.println("Fall detection sequence complete. Returning to monitoring.");
  
  // Reset fall detection
  fallDetected = false;
  
  // Return to monitoring state
  currentState = STATE_MONITORING;
  stateStartTime = millis();
  lastMotionTime = millis();
  lastDebugOutputTime = millis();
}

void checkSleepToggle() {
  // If switch is held down for 3+ seconds, toggle sleep mode
  static bool buttonPressed = false;
  static unsigned long buttonPressStart = 0;
  
  bool currentButtonState = (digitalRead(SWITCH_PIN) == LOW);
  
  if (currentButtonState && !buttonPressed) {
    // Button just pressed
    buttonPressed = true;
    buttonPressStart = millis();
  } 
  else if (!currentButtonState && buttonPressed) {
    // Button just released
    unsigned long pressDuration = millis() - buttonPressStart;
    buttonPressed = false;
    
    if (pressDuration > 3000) {
      // Long press detected, toggle sleep mode
      sleepEnabled = !sleepEnabled;
      Serial.print("Sleep mode ");
      Serial.println(sleepEnabled ? "ENABLED" : "DISABLED");
      
      // Blink LED to indicate new state
      for (int i = 0; i < (sleepEnabled ? 3 : 6); i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
    }
  }
}

void setup() {
  // Set CPU frequency to save power
  setCpuFrequencyMhz(CPU_FREQUENCY_MHZ);
  
  Serial.begin(115200);
  delay(1000); // Give serial a moment to start
  Serial.println("\n\nFall detector with sleep/wake starting up");
  Serial.println("-------------------------------------");
  
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
  
  // Setup basic MPU parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Set up MPU interrupts
  setupMPUInterrupts();
  
  // Reset flags
  motionDetected = false;
  fallDetected = false;
  justWokeUp = false;
  
  // Turn off WiFi initially
  WiFi.mode(WIFI_OFF);
  
  Serial.println("System ready!");
  Serial.println("Using light sleep mode with motion wake and active monitoring");
  Serial.println("Sleep mode is ENABLED. Long-press button (3s) to toggle.");
  
  // Test if interrupt is working
  testMPUInterrupt();
  
  // Initialize monitoring state
  currentState = STATE_MONITORING;
  stateStartTime = millis();
  lastMotionTime = millis();
  lastDebugOutputTime = millis();
  
  // Enter light sleep after showing startup messages
  Serial.println("Entering initial sleep in 5 seconds...");
  Serial.println("Move the device or press button to wake it up afterward!");
  delay(5000);
  enterLightSleep();
}

void loop() {
  // Check if we just woke up
  if (justWokeUp) {
    justWokeUp = false;
    // Handle immediate button press check after wake-up
    if (digitalRead(SWITCH_PIN) == LOW) {
      Serial.println("Button is still pressed after wake-up");
      // Ensure we don't trigger a false fall detection immediately after wake
      delay(500);
    }
  }
  
  // Check for sleep toggle command (long button press)
  checkSleepToggle();
  
  // State machine for different device states
  switch (currentState) {
    case STATE_SLEEPING:
      // We should not reach this in the loop - sleep is handled by esp_light_sleep_start()
      // If we somehow end up here, go back to sleep
      Serial.println("WARNING: In STATE_SLEEPING but not sleeping! Re-entering sleep...");
      enterLightSleep();
      break;
      
    case STATE_MONITORING:
      // Actively monitor accelerometer data for falls
      handleMonitoringState();
      break;
      
    case STATE_ALERT:
      // Handle fall detection and alerts
      handleAlertState();
      break;
  }
  
  // Short button press - always check for immediate fall detection in any state
  if (!fallDetected && digitalRead(SWITCH_PIN) == LOW && currentState != STATE_ALERT) {
    // Only trigger if button press isn't being used for sleep toggle
    unsigned long pressStart = millis();
    delay(100); // Debounce
    
    // If still pressed but less than 1 second, it's a short press for fall detection
    if (digitalRead(SWITCH_PIN) == LOW) {
      unsigned long pressDuration = millis() - pressStart;
      if (pressDuration < 1000) {
        Serial.println("Manual button pressed - triggering fall detection");
        fallDetected = true;
        currentState = STATE_ALERT;
        stateStartTime = millis();
      }
    }
  }
  
  // Short delay to prevent CPU from being 100% utilized when monitoring
  delay(10);
}