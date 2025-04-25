# Fall Detection Device

https://github.com/toopieare/fall_detection

A smart wearable fall detection system developed as part of CS5272 Project, using accelerometer data to detect falls and send alerts through multiple notification methods.

**Sound-Based Fall Detection:**

A new implementation using audio data to detect falls has been added. See the [Sound-Based Fall Detection README](sound-based/README.md). for more details.

## Problem Statement

Falls represent a critical health risk for elderly individuals, often leading to severe injuries, reduced quality of life, and increased healthcare costs. While there are fall detection solutions for the elderly currently available in the market, the extent of adoption of these solutions is limited.

Some examples of existing gerontech solutions include wall-mounted fall detectors and motion sensor floor mats. Alternative wearable devices with fall detection capabilities that don't require physical installation, such as smartwatches, are often too expensive for the elderly. Additionally, their reliance on battery power requires regular charging, which may be difficult for elderly users to remember.

Our project addresses these challenges using embedded systems technologies, combining miniature sensors and energy-efficient design to create a solution that can detect falls with high precision while operating for extended periods on a single charge.

## Overview

This project implements a portable fall detection device using an ESP32 microcontroller and an MPU6050 accelerometer/gyroscope. The system can detect falls using machine learning algorithms, trigger local alerts (LED, buzzer), and send wireless notifications via WiFi.

The device is designed with power efficiency in mind, featuring multiple power optimization strategies and sleep modes to extend battery life while maintaining reliable fall detection.

## Features

- **Reliable Fall Detection**: Uses a machine learning model trained on real movement data
- **Multiple Alert Methods**:
  - Visual alert (LED)
  - Audible alert (Buzzer)
  - Remote notification via WiFi (IFTTT webhooks)
- **Power Optimization**:
  - Light sleep mode with interrupt wake-up
  - Dynamic WiFi activation (connects only when needed)
  - Optimized CPU frequency
- **Manual Alert**: Button to manually trigger an alert
- **Customizable Sensitivity**: Adjustable detection thresholds

## Hardware Components

- ESP32-WROOM-32E / ESP32-S3 DEVKITC microcontroller
- MPU6050 / MPU9250 accelerometer and gyroscope
- LED for visual alerts
- Buzzer for audible alerts
- Push button for manual alerts/testing
- 3.7V LiPo battery with TP4056 charging module

### Wiring Diagram

```
ESP32 → MPU6050
3V3 → VCC
GND → GND
21 → SDA
22 → SCL
19 → INT

ESP32 → LED
2 → Anode(+)
GND → Cathode(-)

ESP32 → Buzzer
4 → IN
3V3 → VCC
GND → GND

ESP32 → Button
15 → Leg 1
GND → Leg 2

ESP32 → TP4056
5V → Out+
GND → Out-

TP4056 → 3.7V LiPo
B+ → Red wire (+)
B- → Black wire (-)
```

## Software Architecture

The device operates in three main states:

1. **SLEEPING**: Light sleep mode to save power, waiting for motion
2. **MONITORING**: Actively monitoring accelerometer data for falls
3. **ALERT**: Sending alerts after fall detection

### Machine Learning for Fall Detection

Our project implements machine learning techniques to improve the reliability of fall detection beyond simple threshold-based approaches.

### Data Collection

We collected movement data for both falls and non-fall activities:

**Accelerometer data:**

- **Non-falls**: Walking, Sitting, Bending down, Hand waving
- **Falls**: Falling backwards, Falling forwards

### Feature Extraction

For each window of accelerometer data (40 samples), we calculate the following features:

1. Mean magnitude of acceleration
2. Standard deviation of magnitude
3. Maximum magnitude
4. Energy (sum of squared magnitudes)

### ML Model Performance

We trained both Logistic Regression and Random Forest models, with Random Forest showing better performance:

**Random Forest Model Results:**

- **Accuracy**: 75.6%
- **F1 Score**: 76.5%
- **Precision (falls)**: 74% (74% of fall alerts are actual falls)
- **Recall (falls)**: 79% (catches 79% of actual falls)

**Feature Importances:**

- **max_magnitude**: 35.1% importance
- **std_magnitude**: 30.3% importance
- **energy**: 17.9% importance
- **mean_magnitude**: 16.7% importance

The machine learning approach significantly improved detection reliability compared to simple threshold-based detection, reducing both false positives (detecting falls when none occurred) and false negatives (missing actual falls).

## Setup and Installation

### Hardware Setup

1. Connect the components according to the wiring diagram above
2. Ensure the battery is charged (connect to a 5V source via the TP4056)

### Software Setup and Compilation

1. **Install Arduino IDE** (version 1.8.19 or later recommended)

   - Download from [Arduino's official website](https://www.arduino.cc/en/software)
   - Install following the instructions for your operating system

2. **Install ESP32 Board Support**:

   - Open Arduino IDE
   - Go to File > Preferences
   - Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to Additional Board URLs field
   - Click OK to save
   - Go to Tools > Board > Boards Manager
   - Search for "ESP32" and install the ESP32 package by Espressif Systems

3. **Install Required Libraries**:

   - Go to Tools > Manage Libraries
   - Search for and install the following libraries:
     - `Adafruit MPU6050` (which automatically installs `Adafruit Sensor`)
     - `Wire` (should be pre-installed)
   - Note: `WiFi` and `HTTPClient` come with the ESP32 board package

4. **Configure Board Settings**:

   - Go to Tools > Board and select "ESP32 Dev Module"
   - Set the following parameters:
     - Flash Mode: "QIO"
     - Flash Frequency: "80MHz"
     - Flash Size: "4MB (32Mb)"
     - Upload Speed: "921600"
     - Core Debug Level: "None"
     - CPU Frequency: "80MHz" (for power savings)

5. **Create the secrets.h file**:

   - In the same directory as your main sketch, create a new file named `secrets.h`
   - Add the following content, replacing the placeholders with your actual information:

   ```cpp
   #define WIFI_SSID "your_wifi_ssid"https://chatgpt.com/c/680b3a34-b230-800b-a4fb-3f145d47d3c9
   #define WIFI_PASSWORD "your_wifi_password"
   #define IFTTT_WEBHOOK_URL "your_ifttt_webhook_url"
   ```

6. **Compile and Upload**:
   - Connect your ESP32 to your computer via USB
   - Select the correct port under Tools > Port
   - Press the Upload button (right arrow icon) or use Sketch > Upload
   - If you encounter upload issues:
     - Make sure the USB cable supports data transfer (not charge-only)
     - Hold the BOOT/FLASH button on the ESP32 during the initial upload stage
     - Some ESP32 boards may require pressing the EN button after initiating upload

### Setting up IFTTT Notifications

1. Create an account on [IFTTT](https://ifttt.com)
2. Create a new Applet:
   - For "If This", select Webhooks and "Receive a web request"
   - Name the event "fall_detected"
   - For "Then That", select Notifications and "Send a notification from the IFTTT app"
   - Set the message to "Fall detected"
3. Get your webhook URL from the Webhooks service page
4. Add this URL to your `secrets.h` file

## Usage

1. **Power On**: The device will blink the LED twice and beep once to indicate it's ready
2. **Initial Mode**: The device will enter sleep mode after 5 seconds to save power
3. **Wake Up**: Any significant motion will wake the device and put it in monitoring mode
4. **Fall Detection**: If a fall is detected:
   - LED will flash rapidly
   - Buzzer will sound an alarm
   - A notification will be sent to your phone via IFTTT
5. **Manual Alert**: Press the button to manually trigger an alert
6. **Sleep Toggle**: Long press the button (3+ seconds) to toggle sleep mode on/off

## Power Consumption

The device has multiple power configurations:

- **Sleep Mode**: ~9.4 mA (using light sleep)
- **Monitoring Mode**: ~38.1 mA
- **Alert Mode**: ~120 mA (peak during WiFi transmission)

Expected battery life:

- With 1000 mAh battery: ~26+ hours in normal operating conditions on mnitoring mode, ~106+ hours in light sleep mode
- Varies based on frequency of motion detection and alerts

## Data Collection and Model Training

This repository includes tools for collecting your own movement data and training a custom fall detection model:

1. **Data Collection**: Use the `ML/data_collection/data_collection.ino` sketch to record movement patterns
2. **Model Training**: Run the Python script `ML/trainer.py` to train a model on your collected data
3. **Model Integration**: The trained model parameters will be saved to `ML/fall_detection_model.h` for use in the main sketch

## Project Files and Structure

The project contains several key files and directories, each with a specific purpose:

```
├── Fall_Detection.ino           # Main Arduino sketch for the fall detection device
├── README.md                    # Documentation and setup instructions
├── secrets.h                    # WiFi and IFTTT credentials (create this file)
├── ML/                          # Machine learning components
│   ├── data_collection/         # Scripts for collecting training data
│   │   └── data_collection.ino  # Arduino sketch for recording movements to SD card
│   ├── trainer.py               # Python script for training the fall detection model
│   ├── fall_detection_model.h   # Generated model parameters for Arduino
│   └── fall_detection_ml/       # Example implementation of ML detection
│       └── fall_detection_ml.ino # Standalone ML-based fall detection implementation which compares Random Forest and Logistic Regression
├── notebook/                    # Jupyter notebooks and related files
│   ├── fall_detection.ipynb     # Jupyter notebook showing model development process
│   ├── sample.ino               # Sample implementation for testing the model
│   └── requirements.txt         # Python dependencies for notebooks
├── data/                        # Training data (CSV files of different movements)
```

### File Descriptions

1. **Fall_Detection.ino**

   - The main Arduino sketch that runs on the ESP32
   - Implements fall detection, sleep mode, WiFi connection, and alert functionality
   - Uses MPU6050 interrupt-based monitoring for power efficiency
   - Contains state machine for handling different device states (sleeping, monitoring, alert)

2. **secrets.h** (you need to create this)

   - Contains sensitive credentials:
     - WiFi SSID and password
     - IFTTT webhook URL for sending notifications
   - Not included in the repository for security reasons

3. **ML/trainer.py**

   - Python script for training the fall detection model
   - Processes CSV files containing accelerometer data
   - Extracts features from movement data
   - Trains and evaluates Logistic Regression and Random Forest models
   - Generates Arduino-compatible code with model parameters
   - Creates visualization of model performance

4. **ML/fall_detection_model.h**

   - Header file containing the trained model parameters
   - Generated by the trainer.py script
   - Includes feature scaling parameters and model coefficients
   - Used by the main sketch to make fall detection decisions

5. **ML/data_collection/data_collection.ino**

   - Arduino sketch for collecting movement data
   - Configures the ESP32 to record accelerometer data to an SD card
   - Includes user interface (buttons, LED) for selecting actions and controlling recording
   - Creates properly formatted CSV files for training the model

6. **ML/fall_detection_ml/fall_detection_ml.ino**

   - Standalone implementation of the ML-based fall detection
   - Useful for testing and validating the ML model
   - Includes emergency button functionality and alerting mechanism

7. **notebook/fall_detection.ipynb**

   - Jupyter notebook showing the data analysis and model development process
   - Contains code for processing accelerometer data
   - Implements feature extraction algorithms
   - Trains and evaluates the initial logistic regression model
   - Includes visualizations and performance metrics

8. **notebook/sample.ino**

   - Example Arduino code showing how to use the fall detection model
   - Provides a simpler implementation for testing purposes
   - Demonstrates feature extraction and classification

9. **notebook/requirements.txt**

   - Lists Python libraries required for the notebook and training scripts
   - Includes pandas, numpy, and scikit-learn

10. **data/** directory
    - Contains CSV files with recorded movement data
    - Organized by movement type (falling_forward, falling_backwards, walking, etc.)
    - Used for training and evaluating the fall detection model

## Project Information

**CS5272 Project - Group 10**

**Team Members:**

- Raymond Daniel Alix LU
- Henry NG Siong Hock
- Joy NG Jing Ru
- Kelvin SOH Boon Kai
