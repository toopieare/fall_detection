
// Fall detection model coefficients
// Trained on 2025-04-20 08:46:48

// Feature order: mean_magnitude, std_magnitude, max_magnitude, energy
const float MODEL_WEIGHTS[4] = { -0.09054448f, 0.00813667f, 0.04179285f, 0.00001560f };
const float MODEL_BIAS = -0.00065902f;

// Function to predict if a set of features indicates a fall
bool isFall(const float *features) {
  float score = MODEL_BIAS;
  for (int i = 0; i < 4; i++) {
    score += MODEL_WEIGHTS[i] * features[i];
  }
  
  // If score > 0, predict fall (probability > 0.5)
  return score > 0;
}
