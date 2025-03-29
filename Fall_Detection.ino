// BLE - Power-Optimized Version

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "secrets.h"

// BLE libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

Adafruit_MPU6050 mpu;

// State variables
bool fallDetected = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;
volatile bool motionDetected = false;
unsigned long lastSensorReadTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastLedToggleTime = 0;
bool ledState = false;

// BLE server name
#define DEVICE_NAME "Fall_Detection_Device"

// UUID for the BLE service and characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Callback for when a device connects or disconnects
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

// Callback for receiving data from client
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = String(pCharacteristic->getValue().c_str());

    if (rxValue.length() > 0) {
      Serial.print("Received via BLE: ");
      Serial.println(rxValue);
      
      // Command handling
      if (rxValue == "STATUS") {
        String response = "Device is operational. Monitoring for falls.";
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
      }
    }
  }
};

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
  
  // Initialize Wire (I2C) before BLE to avoid conflicts
  Wire.begin();
  
  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init(DEVICE_NAME);
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  
  // Add the callbacks
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  // Add a descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // iOS compatibility
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE device started. Advertising as '" + String(DEVICE_NAME) + "'");
  
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
  
  Serial.println("System ready - waiting for fall...");
  
  // Initial status message
  pCharacteristic->setValue("System ready - fall detection active");
  
  // Initialize timers
  lastSensorReadTime = millis();
  lastHeartbeatTime = millis();
  lastLedToggleTime = millis();
}

void loop() {
  // Track current time to minimize millis() calls
  unsigned long currentMillis = millis();
  
  // Handle BLE connections
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE Client connected");
    pCharacteristic->setValue("Device connected successfully");
    pCharacteristic->notify();
  }
  
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE Client disconnected, started advertising again");
  }
  
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
      
      // Turn off LED when done
      digitalWrite(LED_PIN, LOW);
      
      // Stop buzzer
      digitalWrite(BUZZER_PIN, LOW);
  
      // Send fall alert via BLE
      String alertMessage = "FALL DETECTED! Emergency alert triggered! Reason: " + reason;
      
      // Always notify, even if not currently connected
      pCharacteristic->setValue(alertMessage.c_str());
      pCharacteristic->notify();
  
      Serial.println("Fall detected and alerted. System reset.");
      
      // Reset fall detection
      fallDetected = false;
    }
  }
  
  // Send a heartbeat message every 30 seconds when connected
  if (deviceConnected && (currentMillis - lastHeartbeatTime > 30000)) {
    String heartbeatMessage = "Device active, monitoring for falls.";
    pCharacteristic->setValue(heartbeatMessage.c_str());
    pCharacteristic->notify();
    Serial.println("Sent heartbeat message");
    lastHeartbeatTime = currentMillis;
  }
  
  // Short delay to prevent CPU from being 100% utilized
  delay(10);
}