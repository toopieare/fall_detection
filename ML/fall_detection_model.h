
// Fall detection model - Simplified from Random Forest
// Trained on 2025-04-20 10:44:39

// Feature scaling parameters (needed for standardization)
const float FEATURE_MEANS[4] = { 26.17842627f, 2.01447860f, 28.79146328f, 78980.42367051f };
const float FEATURE_STDS[4] = { 35.53828754f, 4.70877739f, 35.31088856f, 162331.33219809f };

// The two most important features are: max_magnitude and std_magnitude

// Sensitivity adjustment (can be tuned after deployment)
float SENSITIVITY = 0.0f;  // Higher = more falls detected but more false positives

// Function to standardize features
void standardizeFeatures(float *features) {
  for (int i = 0; i < 4; i++) {
    features[i] = (features[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
  }
}

// Simplified decision function based on random forest results
bool isFall(float *features) {
  // First standardize the features
  standardizeFeatures(features);
  
  // Use the most important features for a simplified model
  float score = 0.0f;
  
  // Weighted sum of important features (based on feature importance)
  score += features[2] * 0.3505f;
  score += features[1] * 0.3034f;
  
  // Apply threshold determined experimentally from the random forest
  return score > (0.5f - SENSITIVITY);
}
