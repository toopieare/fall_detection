// BLE

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "secrets.h"

// Use the built-in ESP32 BLE libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Pin definitions
const int LED_PIN = 2;
const int BUZZER_PIN = 4;
const int SWITCH_PIN = 15;

// Simple fall detection threshold
const float FALL_THRESHOLD = 1.5;   // Acceleration threshold in G

Adafruit_MPU6050 mpu;

// State variables
bool fallDetected = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE server name
#define DEVICE_NAME "Fall_Detection_Device"

// UUID for the BLE service and characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Define a callback for when a device connects or disconnects
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

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial a moment to start
  Serial.println("Serial connection started");
  
  // Set pins as output before doing anything else
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
  
  // Initial status message
  pCharacteristic->setValue("System ready - waiting for fall...");
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
  
  // Switch detection - digitalRead to return LOW when button is pressed
  bool switchPressed = (digitalRead(SWITCH_PIN) == LOW);
  
  // Simple fall detection OR if switch is pressed
  if ((accGForce >= FALL_THRESHOLD || switchPressed) && !fallDetected) {
    if (switchPressed) {
      Serial.println("TEST BUTTON PRESSED - Simulating fall!");
    } else {
      Serial.println("FALL DETECTED! Acceleration: " + String(accGForce) + " G");
    }

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
    String alertMessage = "FALL DETECTED! Emergency alert triggered!";
    
    // Only send if connected
    if (deviceConnected) {
      pCharacteristic->setValue(alertMessage.c_str());
      pCharacteristic->notify();
    }
    
    Serial.println("Fall detected and alerted. System reset.");
    
    // Reset fall detection
    fallDetected = false;
  }
  
  // Handle connection changes
  if (deviceConnected && !oldDeviceConnected) {
    // Connected
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE Client connected");
    pCharacteristic->setValue("Device connected successfully");
    pCharacteristic->notify();
  }
  
  if (!deviceConnected && oldDeviceConnected) {
    // Disconnected
    delay(500); // Give the Bluetooth stack time to get ready
    pServer->startAdvertising(); // Restart advertising
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE Client disconnected, started advertising again");
  }
  
  delay(100);
}