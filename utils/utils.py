from typing import List
from pynput.keyboard import Key, Listener
import pyaudio
import wave


class Recorder:
    def __init__(self, channels: int = 2, record_chunk: int = 1024, sample_format: int = pyaudio.paInt16,
                 frequency_hz=44100) -> None:
        self.recording: bool = False
        self.frames: List[bytes] = []
        self.channels: int = channels
        self.chunk: int = record_chunk  # Record in chunks of 'chunk' samples (default: 1024)
        self.format: int = sample_format  # Bits per sample (default: 16)
        self.rate: int = frequency_hz  # Record at 'rate' samples per second (default 44.1 kHz)
        self.port = pyaudio.PyAudio()

    def __del__(self):
        self.port.terminate()

    def record_for(self, seconds: int) -> List[bytes]:
        stream = self.port.open(format=self.format, channels=self.channels, rate=self.rate,
                                frames_per_buffer=self.chunk,
                                input=True)
        self.recording = True
        self.frames = [stream.read(self.chunk) for _ in range(0, int(self.rate / self.chunk * seconds))]
        stream.stop_stream()
        stream.close()
        self.recording = False

        return self.frames

    def hold_to_record(self, key: Key) -> List[bytes]:
        def on_press(current_key: Key):
            if not self.recording and current_key == key:
                self.recording = True
                while self.recording:
                    self.frames.append(stream.read(self.chunk))
                stream.stop_stream()
                stream.close()
                return False

        def on_release(current_key: Key):
            if current_key == key:
                self.recording = False
                return False

        self.frames = []
        stream = self.port.open(format=self.format, channels=self.channels, rate=self.rate,
                                frames_per_buffer=self.chunk,
                                input=True)
        with Listener(on_release=on_release) as release_listener:
            with Listener(on_press=on_press) as press_listener:
                press_listener.join()
            release_listener.join()

        return self.frames

    def save_recording(self, filename: str):
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.port.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(self.frames))
