import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score, f1_score
from sklearn.model_selection import train_test_split

# Configuration
DATA_DIR = 'data'  # Directory containing your CSV files
WINDOW_SIZE = 40   # Window size for feature extraction (match Arduino code)
STEP_SIZE = 20     # Step size for sliding window (50% overlap for training data)
TEST_SIZE = 0.2    # Portion of data to use for testing

# Create data directory if it doesn't exist
os.makedirs(DATA_DIR, exist_ok=True)

def extract_features_from_window(x, y, z):
    """
    Extract features from a window of accelerometer data
    """
    # Compute magnitude of acceleration
    mag = np.sqrt(x**2 + y**2 + z**2)
    
    # Extract features
    mean_mag = np.mean(mag)
    std_mag = np.std(mag)
    max_mag = np.max(mag)
    energy = np.sum(mag**2)
    
    return {
        "mean_magnitude": mean_mag,
        "std_magnitude": std_mag,
        "max_magnitude": max_mag,
        "energy": energy
    }

def process_csv_file(file_path, window_size=WINDOW_SIZE, step_size=STEP_SIZE):
    """
    Process a single CSV file and extract features using sliding windows
    """
    # Read CSV file
    df = pd.read_csv(file_path)
    
    # Extract action from filename
    action = os.path.basename(file_path).split('_')[0]
    
    # Check if we have enough samples
    if len(df) < window_size:
        print(f"Warning: {file_path} has fewer than {window_size} samples, skipping.")
        return None
    
    features_list = []
    
    # Process data in sliding windows
    for start in range(0, len(df) - window_size + 1, step_size):
        window = df.iloc[start:start + window_size]
        features = extract_features_from_window(
            window['X'].values, 
            window['Y'].values, 
            window['Z'].values
        )
        features['action'] = action
        features_list.append(features)
    
    return pd.DataFrame(features_list)

def load_and_process_data(data_dir=DATA_DIR):
    """
    Load all CSV files from the data directory and process them
    """
    all_data = []
    
    # Look for all CSV files in the data directory
    for file in os.listdir(data_dir):
        if file.endswith('.csv'):
            file_path = os.path.join(data_dir, file)
            print(f"Processing {file}...")
            features_df = process_csv_file(file_path)
            if features_df is not None:
                all_data.append(features_df)
    
    # Combine all the processed data
    if not all_data:
        raise ValueError("No valid data files found!")
    
    return pd.concat(all_data, ignore_index=True)

def train_model(df):
    """
    Train a logistic regression model on the processed data
    """
    # Convert actions to binary (fall vs. non-fall)
    df['is_fall'] = df['action'].apply(lambda x: 1 if "FALL" in x else 0)
    
    # Balance dataset (optional)
    fall_samples = df[df['is_fall'] == 1]
    non_fall_samples = df[df['is_fall'] == 0]
    
    # If one class dominates, undersample it
    min_count = min(len(fall_samples), len(non_fall_samples))
    
    if len(fall_samples) > min_count:
        fall_samples = fall_samples.sample(min_count, random_state=42)
    if len(non_fall_samples) > min_count:
        non_fall_samples = non_fall_samples.sample(min_count, random_state=42)
    
    balanced_df = pd.concat([fall_samples, non_fall_samples])
    
    # Shuffle the data
    balanced_df = balanced_df.sample(frac=1, random_state=42).reset_index(drop=True)
    
    # Print dataset stats
    print("\nDataset Statistics:")
    print(f"Total samples: {len(balanced_df)}")
    print(f"Fall samples: {len(balanced_df[balanced_df['is_fall'] == 1])}")
    print(f"Non-fall samples: {len(balanced_df[balanced_df['is_fall'] == 0])}")
    
    # Split into features and target
    X = balanced_df[['mean_magnitude', 'std_magnitude', 'max_magnitude', 'energy']]
    y = balanced_df['is_fall']
    
    # Split into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=TEST_SIZE, random_state=42, stratify=y
    )
    
    # Train logistic regression model
    model = LogisticRegression(class_weight='balanced')
    model.fit(X_train, y_train)
    
    # Evaluate model
    y_pred = model.predict(X_test)
    accuracy = accuracy_score(y_test, y_pred)
    f1 = f1_score(y_test, y_pred)
    
    print("\nModel Performance:")
    print(f"Accuracy: {accuracy:.4f}")
    print(f"F1 Score: {f1:.4f}")
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))
    
    # Plot confusion matrix
    cm = confusion_matrix(y_test, y_pred)
    plt.figure(figsize=(8, 6))
    plt.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
    plt.title('Confusion Matrix')
    plt.colorbar()
    tick_marks = np.arange(2)
    plt.xticks(tick_marks, ['Non-Fall', 'Fall'], rotation=45)
    plt.yticks(tick_marks, ['Non-Fall', 'Fall'])
    
    # Add text annotations to confusion matrix
    thresh = cm.max() / 2
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            plt.text(j, i, format(cm[i, j], 'd'),
                    horizontalalignment="center",
                    color="white" if cm[i, j] > thresh else "black")
    
    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')
    plt.savefig('confusion_matrix.png')
    
    return model

def generate_arduino_code(model):
    """
    Generate Arduino code with the trained model coefficients
    """
    coefficients = model.coef_[0]
    intercept = model.intercept_[0]
    
    print("\nModel Coefficients:")
    print(f"Weights: {coefficients}")
    print(f"Bias: {intercept}")
    
    arduino_code = f"""
// Fall detection model coefficients
// Trained on {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}

// Feature order: mean_magnitude, std_magnitude, max_magnitude, energy
const float MODEL_WEIGHTS[4] = {{ {coefficients[0]:.8f}f, {coefficients[1]:.8f}f, {coefficients[2]:.8f}f, {coefficients[3]:.8f}f }};
const float MODEL_BIAS = {intercept:.8f}f;

// Function to predict if a set of features indicates a fall
bool isFall(const float *features) {{
  float score = MODEL_BIAS;
  for (int i = 0; i < 4; i++) {{
    score += MODEL_WEIGHTS[i] * features[i];
  }}
  
  // If score > 0, predict fall (probability > 0.5)
  return score > 0;
}}
"""
    
    # Save Arduino code to a file
    with open('fall_detection_model.h', 'w') as f:
        f.write(arduino_code)
    
    print("\nArduino code saved to 'fall_detection_model.h'")
    return arduino_code

def visualize_features(df):
    """
    Create visualizations of the features to understand their distribution
    """
    # Add binary fall/non-fall column for plotting
    df['is_fall'] = df['action'].apply(lambda x: 1 if "FALL" in x else 0)
    
    # Set up the figure
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot mean magnitude
    axes[0, 0].scatter(
        df.index, df['mean_magnitude'], 
        c=df['is_fall'], cmap='coolwarm', alpha=0.7
    )
    axes[0, 0].set_title('Mean Magnitude')
    axes[0, 0].set_ylabel('Value')
    
    # Plot standard deviation
    axes[0, 1].scatter(
        df.index, df['std_magnitude'], 
        c=df['is_fall'], cmap='coolwarm', alpha=0.7
    )
    axes[0, 1].set_title('Standard Deviation')
    
    # Plot max magnitude
    axes[1, 0].scatter(
        df.index, df['max_magnitude'], 
        c=df['is_fall'], cmap='coolwarm', alpha=0.7
    )
    axes[1, 0].set_title('Max Magnitude')
    axes[1, 0].set_xlabel('Sample Index')
    axes[1, 0].set_ylabel('Value')
    
    # Plot energy
    axes[1, 1].scatter(
        df.index, df['energy'], 
        c=df['is_fall'], cmap='coolwarm', alpha=0.7
    )
    axes[1, 1].set_title('Energy')
    axes[1, 1].set_xlabel('Sample Index')
    
    # Add a color bar to show fall/non-fall mapping
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', 
               markersize=10, label='Non-Fall'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
               markersize=10, label='Fall')
    ]
    fig.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, 0.05), 
               ncol=2, frameon=False)
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)
    plt.savefig('feature_visualization.png')
    print("Feature visualization saved to 'feature_visualization.png'")

def main():
    print("Fall Detection Model Training")
    print("============================")
    
    # Load and process the data
    print("\nLoading and processing data...")
    try:
        processed_data = load_and_process_data()
        print(f"Processed {len(processed_data)} samples.")
        
        # Visualize the features
        print("\nGenerating feature visualizations...")
        visualize_features(processed_data)
        
        # Train the model
        print("\nTraining model...")
        model = train_model(processed_data)
        
        # Generate Arduino code
        print("\nGenerating Arduino code...")
        generate_arduino_code(model)
        
        print("\nTraining complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        return
    
if __name__ == "__main__":
    main()