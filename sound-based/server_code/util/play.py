import pyaudio

# Open the audio stream
p = pyaudio.PyAudio()

# Parameters (adjust these depending on your audio data)
FORMAT = pyaudio.paInt16  # Typically 16-bit PCM data
CHANNELS = 1              # Mono (1) or Stereo (2)
RATE = 16000              # Sample rate (samples per second)
CHUNK = 1024              # Size of each chunk to play

# Open an audio stream
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                output=True,
                frames_per_buffer=CHUNK)

# Read binary data from a file or WebSocket (as per your setup)
with open("NoFall5.pcm", "rb") as f:
    while True:
        # Read the next chunk of data
        data = f.read(CHUNK)
        if not data:
            break
        # Play the audio data
        stream.write(data)

# Close the stream and terminate PyAudio
stream.stop_stream()
stream.close()
p.terminate()
