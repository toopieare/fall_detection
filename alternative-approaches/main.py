import tensorflow as tf
import numpy as np
import scipy
import socket
import threading
import io
import librosa
from fastapi import FastAPI
from scipy.io import wavfile

class FallDetector:
    def __init__(self, model_path="sound-detection.h5", desired_sample_rate=16000, n_mfcc=13):
        # load the pretrained model
        self.model = tf.keras.models.load_model(model_path)
        print(f"[INIT] Loaded model from {model_path}, input shape {self.model.input_shape}")

        self.desired_sample_rate = desired_sample_rate
        self.n_mfcc = n_mfcc

    def _ensure_sample_rate(self, original_sr, waveform):
        if original_sr != self.desired_sample_rate:
            desired_length = int(round(len(waveform) / original_sr * self.desired_sample_rate))
            waveform = scipy.signal.resample(waveform, desired_length)
        return self.desired_sample_rate, waveform

    def _load_wav(self, file_or_path):
        if hasattr(file_or_path, "read"):
            file_obj = file_or_path
            file_obj.seek(0)
            data, sample_rate = librosa.load(file_obj, sr=self.desired_sample_rate)
        else:
            data, sample_rate = librosa.load(file_or_path, sr=self.desired_sample_rate)
        return data, sample_rate

    def detect_fall(self, file_or_path):
        # load audio
        data, sr = self._load_wav(file_or_path)

        # extract features
        mfccs = librosa.feature.mfcc(y=data.astype(np.float32), sr=sr, n_mfcc=self.n_mfcc)
        features = np.mean(mfccs, axis=1)

        # prepare model input
        x = features.reshape(1, -1)

        # interpret results
        preds = self.model.predict(x)
        if preds.ndim == 2 and preds.shape[1] == 1:
            label = "fall" if preds[0,0] > 0.5 else "no fall"
        elif preds.ndim == 2 and preds.shape[1] == 2:
            label = "fall" if np.argmax(preds[0]) == 1 else "no fall"
        else:
            label = "fall" if np.argmax(preds) == 1 else "no fall"

        return label

app = FastAPI()

detector = FallDetector(model_path="sound-detection.h5")

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
SAMPLE_RATE = detector.desired_sample_rate
SAMPLE_WIDTH = 2
CHUNK_SECONDS = 4
CHUNK_BYTES = SAMPLE_RATE * SAMPLE_WIDTH * CHUNK_SECONDS


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[UDP] Listening on port {UDP_PORT}…")

    buffer = bytearray()
    while True:
        data, addr = sock.recvfrom(4096)
        print(f"[UDP] Received {len(data)} bytes from {addr}")
        buffer.extend(data)

        while len(buffer) >= CHUNK_BYTES:
            chunk = buffer[:CHUNK_BYTES]
            del buffer[:CHUNK_BYTES]

            wav_buf = io.BytesIO()
            wavfile.write(wav_buf, SAMPLE_RATE, np.frombuffer(chunk, dtype=np.int16))
            wav_buf.seek(0)

            prediction = detector.detect_fall(wav_buf)
            print(f"[DETECT] {prediction}")

@app.on_event("startup")
def start_udp_thread():
    thread = threading.Thread(target=udp_listener, daemon=True)
    thread.start()

