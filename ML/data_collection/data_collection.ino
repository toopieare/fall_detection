#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>

// Define pins
#define MODE_BUTTON_PIN 15        // Button to cycle through actions
#define RECORD_BUTTON_PIN 4       // Button to start/stop recording
#define LED_PIN 2                 // LED to indicate recording status
#define BUZZER_PIN 14             // Buzzer pin - change to your actual buzzer pin
#define SD_CS_PIN 5               // CS pin for SD card module
#define MPU_INT_PIN 13            // Changed from 19 to avoid conflict with MISO

// Define actions to record
enum Action {
  BENDING_DOWN,
  FALLING_BACKWARDS,
  FALLING_FORWARD,
  HAND_WAVING,
  SITTING,
  WALKING,
  NUM_ACTIONS  // Used to cycle through actions
};

// Convert action enum to string
const char* actionToString(Action action) {
  switch(action) {
    case BENDING_DOWN: return "BENDING_DOWN";
    case FALLING_BACKWARDS: return "FALLING_BACKWARDS";
    case FALLING_FORWARD: return "FALLING_FORWARD";
    case HAND_WAVING: return "HAND_WAVING";
    case SITTING: return "SITTING";
    case WALKING: return "WALKING";
    default: return "UNKNOWN";
  }
}

// Current recording settings
Action currentAction = BENDING_DOWN;  // Start with first action
bool isRecording = false;
int recordingNumber = 1;
unsigned long recordingStartTime = 0;
bool sdCardInitialized = false;

// Button state tracking
bool lastModeButtonState = HIGH;
bool lastRecordButtonState = HIGH;

// LED patterns for different actions (to indicate which action is selected)
// Number of blinks corresponds to action number (1-6)
const int actionBlinks[] = {1, 2, 3, 4, 5, 6};

// SD card file
File dataFile;

// Accelerometer
Adafruit_MPU6050 mpu;
bool mpuInitialized = false;

// Function prototypes
void blinkActionPattern();
void nextAction();
void startRecording();
void stopRecording();
void silenceBuzzer();

// Make sure buzzer is off
void silenceBuzzer() {
  // Using digitalWrite for simple on/off buzzer
  digitalWrite(BUZZER_PIN, LOW);
  
  // If using tone() function, use noTone()
  noTone(BUZZER_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial monitor time to open
  
  Serial.println("\n\n-----------------------------");
  Serial.println("ESP32 Data Collection System");
  Serial.println("-----------------------------");
  
  // Setup pins
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initial buzzer state
  digitalWrite(BUZZER_PIN, LOW);
  noTone(BUZZER_PIN);
  Serial.println("Buzzer initialized");
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    mpuInitialized = false;
    
    // Error pattern - rapid blinking
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  } else {
    mpuInitialized = true;
    Serial.println("MPU6050 initialized successfully");
    
    // Configure accelerometer
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Initialize SD card
  Serial.println("Initializing SD card...");
  Serial.print("Using CS pin: ");
  Serial.println(SD_CS_PIN);
  
  // Try initializing up to 3 times
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("Attempt ");
    Serial.print(attempt);
    Serial.println(" to initialize SD card...");
    
    if (SD.begin(SD_CS_PIN)) {
      sdCardInitialized = true;
      Serial.println("SD card initialized successfully!");
      break;
    } else {
      Serial.println("SD card initialization failed!");
      delay(1000);  // Wait before retrying
    }
  }
  
  if (!sdCardInitialized) {
    Serial.println("All attempts to initialize SD card failed!");
    
    // Error pattern - double blink
    while (1) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(500);
    }
  }
  
  // Success indication - one long blink
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  
  // Beep once to indicate success
  tone(BUZZER_PIN, 2000);
  delay(200);
  noTone(BUZZER_PIN);
  
  Serial.println("System ready!");
  Serial.print("Current action: ");
  Serial.println(actionToString(currentAction));
  
  // Show current action pattern
  blinkActionPattern();
}

void blinkActionPattern() {
  Serial.print("Showing pattern for action: ");
  Serial.println(actionToString(currentAction));
  
  delay(500);  // Pause before starting pattern
  
  // Blink the number of times corresponding to the action
  int blinks = actionBlinks[currentAction];
  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  delay(500);  // Pause after pattern
}

void nextAction() {
  // Move to next action
  currentAction = static_cast<Action>((currentAction + 1) % NUM_ACTIONS);
  recordingNumber = 1;  // Reset recording count for new action
  
  Serial.print("Action changed to: ");
  Serial.println(actionToString(currentAction));
  
  // Show the selected action via LED pattern
  blinkActionPattern();
}

void startRecording() {
  if (!sdCardInitialized) {
    Serial.println("Cannot start recording: SD card not initialized");
    return;
  }
  
  // Create filename with action and recording number
  String filename = String(actionToString(currentAction)) + "_" + String(recordingNumber) + ".csv";
  
  // Open file
  Serial.print("Opening file: ");
  Serial.println(filename);
  dataFile = SD.open("/" + filename, FILE_WRITE);
  
  if (!dataFile) {
    Serial.println("Error opening file!");
    return;
  }
  
  // Write header
  dataFile.println("TIME,X,Y,Z,ACTION");
  
  isRecording = true;
  recordingStartTime = millis();
  digitalWrite(LED_PIN, HIGH);  // Solid LED while recording
  Serial.println("Recording started");
  
  // Short beep to indicate recording started
  tone(BUZZER_PIN, 2000);
  delay(100);
  noTone(BUZZER_PIN);
}

void stopRecording() {
  if (isRecording) {
    dataFile.close();
    isRecording = false;
    recordingNumber++;
    digitalWrite(LED_PIN, LOW);
    
    // Indicate recording stopped with 2 long blinks
    digitalWrite(LED_PIN, HIGH); delay(300);
    digitalWrite(LED_PIN, LOW);  delay(150);
    digitalWrite(LED_PIN, HIGH); delay(300);
    digitalWrite(LED_PIN, LOW);  delay(150);
    
    Serial.println("Recording stopped");
    Serial.print("Next recording number: ");
    Serial.println(recordingNumber);
    
    // Two short beeps to indicate recording stopped
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, 2000);
      delay(100);
      noTone(BUZZER_PIN);
      delay(100);
    }
  }
}

void recordAccelerometerData() {
  if (!mpuInitialized) {
    return;
  }
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long timestamp = millis() - recordingStartTime;
  
  // Format: TIME,X,Y,Z,ACTION
  String dataString = String(timestamp) + "," + 
                     String(a.acceleration.x) + "," +
                     String(a.acceleration.y) + "," +
                     String(a.acceleration.z) + "," +
                     actionToString(currentAction);
  
  dataFile.println(dataString);
  
  // Print data every 500ms for debugging
  if (timestamp % 500 == 0) {
    Serial.println(dataString);
  }
}

// Simplified button handling similar to the test sketch
void handleButtons() {
  // Read current button states
  bool modeButtonState = digitalRead(MODE_BUTTON_PIN);
  bool recordButtonState = digitalRead(RECORD_BUTTON_PIN);
  
  // Mode button handling (simplified)
  if (modeButtonState != lastModeButtonState) {
    delay(50);  // Simple debounce
    if (modeButtonState == LOW && lastModeButtonState == HIGH) {
      Serial.println("Mode button pressed!");
      if (!isRecording) {  // Only change action if not recording
        nextAction();
      } else {
        Serial.println("Cannot change action while recording");
      }
    }
    lastModeButtonState = modeButtonState;
  }
  
  // Record button handling (simplified)
  if (recordButtonState != lastRecordButtonState) {
    delay(50);  // Simple debounce
    if (recordButtonState == LOW && lastRecordButtonState == HIGH) {
      Serial.println("Record button pressed!");
      if (isRecording) {
        stopRecording();
      } else {
        startRecording();
      }
    }
    lastRecordButtonState = recordButtonState;
  }
}

void loop() {
  // Check buttons first - simplified handling
  handleButtons();
  
  // Make sure buzzer isn't stuck on
  static unsigned long lastBuzzerCheck = 0;
  if (millis() - lastBuzzerCheck > 5000) {  // Check every 5 seconds
    silenceBuzzer();
    lastBuzzerCheck = millis();
  }
  
  // Record data if active
  if (isRecording) {
    recordAccelerometerData();
    
    // Flash LED rapidly during recording as visual feedback
    if ((millis() - recordingStartTime) % 200 < 100) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  
  // Short delay to avoid overloading the processor
  delay(10);
}