from fastapi import FastAPI
import socket
import threading
import os
import time
from audio_fall_detector import FallDetector
import wave

app = FastAPI()
detector = FallDetector()

UDP_IP = "0.0.0.0"   # Accept from any IP
UDP_PORT = 5005      # Must match the ESP32's UDP port
PCM_FILE = "Fall.pcm"
WAV_FILE = "Fall.wav"
SAMPLE_RATE = 16000  # Hz
CHANNELS = 1
SAMPLE_WIDTH = 2  # bytes (16-bit PCM)

def udp_listener():
    print(f"[UDP] Listening on {UDP_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    i = 0
    while True:
        data, addr = sock.recvfrom(1024)  # Adjust buffer size if needed
        with open(PCM_FILE, "ab") as f:
            print("Write")
            f.write(data)
        print(f"[UDP] Received {len(data)} bytes from {addr} [Chunk {i}]")
        i += 1


def pcm_to_wav():
    while True:
        # Run the Conversion every 7 seconds
        time.sleep(7)
        if os.path.exists(PCM_FILE):
            try:
                print("[WAV] Converting PCM to WAV...")
                with open(PCM_FILE, 'rb') as pcm:
                    pcm_data = pcm.read()

                # Convert PCM to WAV
                with wave.open(WAV_FILE, 'wb') as wav:
                    wav.setnchannels(CHANNELS)
                    wav.setsampwidth(SAMPLE_WIDTH)
                    wav.setframerate(SAMPLE_RATE)
                    wav.writeframes(pcm_data)

                print(f"WAV file saved to {WAV_FILE}")

                # Detect fall
                prediction = detector.detect_fall(WAV_FILE)
                print(f"Prediction: {prediction}")
                # For DEMO
                # if(prediction == "fall"):
                #     print("Fall detected!")
                #     os._exit(0)

            except Exception as e:
                print(f"[WAV] Error converting PCM to WAV: {e}")

@app.on_event("startup")
def start_udp_thread():
    print("[UDP] Starting UDP listener thread...")
    thread = threading.Thread(target=udp_listener, daemon=True)
    thread.start()

    print("[WAV] Starting WAV converter thread...")
    wav_thread = threading.Thread(target=pcm_to_wav, daemon=True)
    wav_thread.start()