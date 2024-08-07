from typing import List, Optional
from typing_extensions import Self
from pynput.keyboard import Key, Listener
import pyaudio
import wave
import asyncio


class Recorder:
    def __init__(self, channels: int = 2, record_chunk: int = 1024, sample_format: int = pyaudio.paInt16, frequency_hz=44100) -> None:
        self.frames: List[bytes] = []
        self.channels: int = channels
        self.chunk: int = record_chunk  # Record in chunks of 'chunk' samples (default: 1024)
        self.format: int = sample_format  # Bits per sample (default: 16)
        self.rate: int = frequency_hz  # Record at 'rate' samples per second (default 44.1 kHz)
        self.port = pyaudio.PyAudio()

        self.recording: bool = False
        self._recording_task: Optional[asyncio.Task] = None

    def __del__(self) -> None:
        self.port.terminate()

    def __enter__(self) -> Self:
        return self

    async def __aenter__(self) -> Self:
        self._recording_task = asyncio.create_task(self._async_record())
        return self.__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        pass

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.stop_recording()
        self.__exit__(exc_type, exc_val, exc_tb)

    def _record(self) -> None:
        stream = self.port.open(format=self.format, channels=self.channels, rate=self.rate, frames_per_buffer=self.chunk, input=True)
        self.frames = []
        self.recording = True
        while self.recording: self.frames.append(stream.read(self.chunk))
        stream.stop_stream()
        stream.close()

    async def _async_record(self) -> None:
        stream = self.port.open(format=self.format, channels=self.channels, rate=self.rate, frames_per_buffer=self.chunk, input=True)
        self.frames = []
        self.recording = True
        while self.recording:
            self.frames.append(stream.read(self.chunk))
            await asyncio.sleep(0.001)
        stream.stop_stream()
        stream.close()

    async def start_recording(self):
        if not self.recording:
            # if self._recording_thread.is_alive(): self._recording_thread.join()
            # self._recording_thread.start()
            await self._recording_task

    async def stop_recording(self):
        self.recording = False
        while not self._recording_task.done(): await asyncio.sleep(0.001)
        # if self._recording_thread.is_alive(): self._recording_thread.join()

    def record_for(self, seconds: int) -> List[bytes]:
        stream = self.port.open(format=self.format, channels=self.channels, rate=self.rate, frames_per_buffer=self.chunk, input=True)
        self.recording = True
        self.frames = [stream.read(self.chunk) for _ in range(0, int(self.rate / self.chunk * seconds))]
        stream.stop_stream()
        stream.close()
        self.recording = False

        return self.frames

    #TODO(hold-to-record): This doesn't appear to be implemented correctly. The self._record() method finishes the loop correctly,
    #   closes the stream and exits. Both joined listeners seem to finish correctly and hold_to_record() exits. But after self.__exit__()
    #   is executed self.__del__() is not called. If Recorder is instantiated inside a loop, self.__del__() is never called normally
    #   during execution, but all calls are performed only when the program exits/interrupts.
    def hold_to_record(self, key: Key) -> List[bytes]:
        def on_press(current_key: Key):
            if not self.recording and current_key == key:
                self._record()
                return False

        def on_release(current_key: Key):
            if current_key == key:
                self.recording = False
                return False

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
