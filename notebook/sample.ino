#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <vector>
#include <cmath> // sqrtf, powf, etc.

Adafruit_MPU6050 mpu;

// We'll store accelerometer samples here:
static std::vector<float> x;
static std::vector<float> y;
static std::vector<float> z;

constexpr int NUM_SAMPLES = 40; // how many samples per "window"

struct Features {
  float mean_magnitude;
  float max_magnitude;
  float std_magnitude;  // standard deviation of magnitude
  float energy;
};

// Extract stats from x,y,z in the current "window"
Features extract_features_from_window(const std::vector<float>& xv,
                                      const std::vector<float>& yv,
                                      const std::vector<float>& zv) {
  // Safety check: must have equal and non-zero sizes
  size_t n = xv.size();
  if (yv.size() != n || zv.size() != n || n == 0) {
    // Return dummy or handle error
    return {0.f, 0.f, 0.f, 0.f};
  }

  // 1. Compute magnitude for each sample
  //    mag[i] = sqrt(x^2 + y^2 + z^2)
  std::vector<float> mag(n);
  for (size_t i = 0; i < n; i++) {
    float xx = xv[i];
    float yy = yv[i];
    float zz = zv[i];
    mag[i] = sqrtf(xx*xx + yy*yy + zz*zz);
  }

  // 2. Mean magnitude
  float sum_mag = 0.f;
  for (auto val : mag) {
    sum_mag += val;
  }
  float mean_mag = sum_mag / static_cast<float>(n);

  // 3. Max magnitude
  float max_mag = mag[0];
  for (auto val : mag) {
    if (val > max_mag) {
      max_mag = val;
    }
  }

  // 4. Compute standard deviation of magnitude
  //    var = average( (mag[i] - mean_mag)^2 )
  //    std = sqrt(var)
  float sum_sq_diff = 0.f;
  for (auto val : mag) {
    float diff = val - mean_mag;
    sum_sq_diff += (diff * diff);
  }
  float var_mag = sum_sq_diff / static_cast<float>(n);
  float std_mag = sqrtf(var_mag);

  // 5. Energy = sum of (mag^2)
  float energy = 0.f;
  for (auto val : mag) {
    energy += (val * val);
  }

  Features feats {
    mean_mag,
    max_mag,
    std_mag,
    energy
  };
  return feats;
}

void setup() {
  Serial1.begin(115200);

  if (!mpu.begin()) {
    Serial1.println("Failed to initialize MPU6050!");
    while (true) {
      delay(100);
    }
  }
  Serial1.println("MPU6050 ready!");

  // You can optionally set the accelerometer range and filter band:
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

const float W[4] = { 0.19782033989850656, -0.025113247090156547, 3.0871287963889946, 0.00255549883106743 };   // 32 weights
const float B = -5.63995471;       // bias

bool isFall(const float *feat) {
    float s = B;
    for (int i=0;i<4;i++)
      s += W[i] * feat[i];
    Serial1.println(s);
    return s > 0;            
}

// Compute features from your accumulated data (see previous code for details)
void computeFeatures(float features[4],
                     const float mean_mag,
                     const float std_mag,
                     const float max_mag,
                     const float energy) {
    // Directly assign the precomputed scalar values.
    features[0] = mean_mag;
    features[1] = std_mag;
    features[2] = max_mag;
    features[3] = energy;
}

void loop() {
  // 1. Read accelerometer
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  // 2. Push the new sample into each vector
  x.push_back(acc.acceleration.x);
  y.push_back(acc.acceleration.y);
  z.push_back(acc.acceleration.z);
  Serial1.println(acc.acceleration.x);
  Serial1.println(acc.acceleration.y);  
  // 3. Check if we have enough samples
  Serial1.println(acc.acceleration.z);  
  if (x.size() >= NUM_SAMPLES) {
    // 4. Extract features from the entire window
    Features feats = extract_features_from_window(x, y, z);

    // 5. Print or use the features
    Serial1.print("Window Features => ");
    Serial1.print("mean_mag: ");
    Serial1.print(feats.mean_magnitude, 3);
    Serial1.print(", max_mag: ");
    Serial1.print(feats.max_magnitude, 3);
    Serial1.print(", std_mag: ");
    Serial1.print(feats.std_magnitude, 3);
    Serial1.print(", energy: ");
    Serial1.println(feats.energy, 3);

    float features[4];
    float computedMean = feats.mean_magnitude; 
    float computedStd  = feats.std_magnitude;
    float computedMax  = feats.max_magnitude;
    float computedEnergy = feats.energy;

    computeFeatures(features, computedMean, computedStd, computedMax, computedEnergy);
    Serial1.println(", Features: ");
    for (int i = 0; i < 4; i++) {
      Serial1.print("Feature ");
      Serial1.print(i);
      Serial1.print(": ");
      Serial1.println(features[i]);
    }
    if (isFall(features)) { // 1 for fall, 0 for no fall
      Serial1.println("Fall detected!");
    } else {
      Serial1.println("No fall detected.");
    }

    // 6. Clear the vectors for the next window
    x.clear();
    y.clear();
    z.clear();

  }

  // Control the sampling rate (e.g., ~100 Hz)
  delay(10);
}