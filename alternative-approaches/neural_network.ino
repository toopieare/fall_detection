#include <Arduino.h>
#include <ArduTFLite.h>
#include "./model.h"

#define MIC_PIN 34                // MAX9814 microphone
#define AUDIO_SAMPLE_RATE 100 
#define AUDIO_WINDOW_DURATION 3 // follows sample duration used in model training
#define NUM_SAMPLES (AUDIO_SAMPLE_RATE * AUDIO_WINDOW_DURATION)
#define NUM_FEATURES 13    

constexpr int kTensorArenaSize = 10 * 1024;
alignas(16) uint8_t tensorArena[kTensorArenaSize];

void capture_audio_features(float* features, int num_features) {
  int audio_buffer[NUM_SAMPLES];

  Serial.println("Capturing audio samples...");
  for (int i = 0; i < NUM_SAMPLES; i++) {
    audio_buffer[i] = analogRead(MIC_PIN);
    delay(1000 / AUDIO_SAMPLE_RATE);
  }

  
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += audio_buffer[i];
  }
  float mean = sum / NUM_SAMPLES;

  float variance = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    variance += (audio_buffer[i] - mean) * (audio_buffer[i] - mean);
  }
  variance /= NUM_SAMPLES;
  float stddev = sqrt(variance);

  features[0] = mean;
  features[1] = stddev;
  for (int i = 2; i < num_features; i++) {
    features[i] = mean + (i * 0.1 * stddev);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MIC_PIN, INPUT);

  Serial.println("Initializing TensorFlow Lite Micro Interpreter...");
  if (!modelInit(fall_detection_model_tflite, tensorArena, kTensorArenaSize)) {
    Serial.println("Model initialization failed!");
    while (true);
  }
  Serial.println("Model initialization done.");
  delay(1000); 
}

void loop() {
  float features[NUM_FEATURES];

  capture_audio_features(features, NUM_FEATURES);

  for (int i = 0; i < NUM_FEATURES; i++) {
    modelSetInput(features[i], i);
  }

  if (!modelRunInference()) {
    Serial.println("RunInference Failed!");
    return;
  }

  float prediction = modelGetOutput(0);
  Serial.print("Prediction value: ");
  Serial.println(prediction);

  if (prediction > 0.5) {
    Serial.println("FALL DETECTED");
  } else {
    Serial.println("No fall detected");
  }
}


