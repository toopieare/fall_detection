import wave

def pcm_to_wav(pcm_file, wav_file, channels=1, sample_rate=16000, sample_width=2):
    with open(pcm_file, 'rb') as pcm:
        pcm_data = pcm.read()

    with wave.open(wav_file, 'wb') as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(sample_width)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm_data)

    print(f"WAV file saved to {wav_file}")

# Example usage
pcm_to_wav("Fall.pcm", "output.wav", channels=1, sample_rate=16000, sample_width=2)