import tensorflow as tf
import tensorflow_hub as hub
import numpy as np
import scipy
import wave
import os
import glob
import re
from scipy.io import wavfile
from scipy.spatial import distance

class FallDetector:
    def __init__(self, audio_dir="Sounds"):
        self.model = hub.load('https://tfhub.dev/google/yamnet/1')
        self.audio_dir = audio_dir
        self.embeddings_fall = self._load_embeddings("Fall", r"Fall\d+\.pcm")
        self.embeddings_nofall = self._load_embeddings("NoFall", r"NoFall\d+\.pcm")

    def _ensure_sample_rate(self, original_sample_rate, waveform, desired_sample_rate=16000):
        if original_sample_rate != desired_sample_rate:
            desired_length = int(round(float(len(waveform)) / original_sample_rate * desired_sample_rate))
            waveform = scipy.signal.resample(waveform, desired_length)
        return desired_sample_rate, waveform

    def _convert_pcm_to_wave(self, pcm_path):
        wav_path = pcm_path.replace(".pcm", ".wav")
        with open(pcm_path, 'rb') as pcmfile:
            pcm_data = pcmfile.read()
        with wave.open(wav_path, 'wb') as wavfile_:
            wavfile_.setnchannels(1)
            wavfile_.setsampwidth(2)
            wavfile_.setframerate(16000)
            wavfile_.writeframes(pcm_data)
        return wav_path

    def _get_embedding(self, file_path):
        if file_path.endswith(".pcm"):
            file_path = self._convert_pcm_to_wave(file_path)

        sample_rate, wav_data = wavfile.read(file_path, 'rb')
        sample_rate, wav_data = self._ensure_sample_rate(sample_rate, wav_data)

        waveform = wav_data / tf.int16.max
        scores, embeddings, spectrogram = self.model(waveform)
        return np.array(tf.reduce_mean(embeddings, axis=0))

    def _load_embeddings(self, prefix, regex):
        pattern = re.compile(regex)
        embeddings = []
        for file in glob.glob(f"{self.audio_dir}/{prefix}*.pcm"):
            filename = os.path.basename(file)
            if pattern.fullmatch(filename):
                emb = self._get_embedding(f"{self.audio_dir}/{filename}")
                embeddings.append(emb)
        return np.array(embeddings)

    def detect_fall(self, file_path):
        query = self._get_embedding(file_path)
        fall_dist = np.mean([distance.cosine(query, emb) for emb in self.embeddings_fall])
        nofall_dist = np.mean([distance.cosine(query, emb) for emb in self.embeddings_nofall])

        return "fall" if fall_dist < nofall_dist else "no fall"

### Usage
# from audio_fall_detector import FallDetector
# detector = FallDetector()
# prediction = detector.detect_fall('output.wav')