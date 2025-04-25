# Fall Detection Device using Sound

This project is a fall detection device that uses sound to detect falls. It is designed to be used in a home environment.

## Overview

This project implements a fall detectiond device using an ESP32 microcontroller with a INMP441(I2S Microphone). The System is designed to collect sound data on the ESP32 and send it to a server for processing. The server uses a machine learning model to detect falls based on the sound data.

## Features

This project includes the following features:

- Sound data collection using an I2S microphone on the ESP32
- A UDP Server to receive sound data from the ESP32
- A machine learning model to detect falls based on sound data

## Hardware Components

- ESP32-S3 DEVKITC Microcontroller
- INMP441 I2S Microphone
- Powerbank

### Wiring Diagram

| ESP32 Pin | INMP441 Pin |
| --------- | ----------- |
| 3.3V      | VDD         |
| GND       | GND         |
| 12        | SCK         |
| 11        | WS          |
| 10        | SD          |

| ESP 32 Port | USB Powerbank Port |
| ----------- | ------------------ |
| Micro USB   | USB-A(5V Output)   |

## Software Architecture

Unlike the Fall Detection implementation using accelerometer data,this implementation utilizes acoustic signals for detecting falls. The device continuously captures ambient sound data, which necessitates keeping the microphone and associated processing components active at all times. As a result, the system cannot enter low-power or deep sleep modes, nor be repurposed for other tasks during operation.

In this implementation, the device operates in a persistent listening state and transmits raw sound data to a remote server via the UDP protocol for minimal latency. The server then processes the incoming audio stream using a machine learning model trained to identify fall-related acoustic patterns.

## Limitations

- The system requires a constant power supply, as it cannot enter low-power or deep sleep modes.

### Data Collection

We collected audio data corresponding to both fall and non-fall activities using an ESP32 microcontroller connected to an INMP441 digital MEMS microphone via the I2S interface. The audio was recorded in Pulse-Code Modulation (PCM) format with a 16-bit depth, 16 kHz sampling rate, and mono channel configuration, ensuring compatibility with audio classification models. The raw PCM files were subsequently converted into WAV format, preserving the original recording specifications. This conversion facilitated easier preprocessing and model compatibility, especially for training with the YAMNet model, which requires 16 kHz mono WAV inputs. Each audio sample was labeled as either "fall" or "non-fall" to support supervised learning.

## Software Setup and Comppilation

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
     - `WiFiUDP`
     - `driver/i2s`

4. **Modify the Code**:

   - Open the `client_code\fall_detection_udp.ino` file in Arduino IDE
   - Update the following parameters in the code:
     - Wi-Fi credentials: Set your network’s SSID and password.
     - UDP server IP address and port number: Ensure they match the server configuration.
     - Sampling rate and bit depth: Adjust the sampling rate and bits per sample as needed for your audio data.
     ```cpp
     const char* ssid = "your_SSID";
     const char* password = "your_PASSWORD";
     const char* udp_host = "192.168.0.4"; // Server IP
     const uint16_t udp_port = 5005; // Server Port
     const uint16_t sampling_rate = 16000; // Sampling rate in Hz
     const uint16_t bits_per_sample = 16; // Bits per sample
     ```

5. **Compile and Upload the Code**:

   - Connect your ESP32 board to your computer
   - Select the correct board and port in Arduino IDE (Tools > Board and Tools > Port)
   - Click on the upload button to compile and upload the code to the ESP32

6. **Building of the Base Station Code**:
   - Run pip install -r requirements.txt to install the required libraries
   - Run the server code using the command:
     ```bash
     python -m uvicorn server:app --host 0.0.0.0
     ```
   - The server will start and listen for incoming UDP packets on the specified port (5005 in this case).

## Usage

1. **Trying to Connect to the Wi-Fi**: RED LED Light
2. **Connected to the Wi-Fi**: GREEN LED Light
3. **Trying to Connect to the Server**: GREEN LED Light
4. **Connected and Transmitting Data**: BLUE LED Light

## Usage

- Once the Flask Server is showing Data is being received, it is an indicator that the device is sending data
