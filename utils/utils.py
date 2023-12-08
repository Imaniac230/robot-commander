def record_audio(seconds: int, filename: str = "output.wav", frequency_hz = 44100):
    import pyaudio
    import wave

    channels = 2
    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paInt16  # 16 bits per sample
    fs = frequency_hz  # Record at given (default 44.1 kHz) samples per second

    p = pyaudio.PyAudio()
    stream = p.open(format=sample_format,
                    channels=channels,
                    rate=fs,
                    frames_per_buffer=chunk,
                    input=True)
    frames = [stream.read(chunk) for _ in range(0, int(fs / chunk * seconds))]
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(sample_format))
    wf.setframerate(fs)
    wf.writeframes(b''.join(frames))
    wf.close()