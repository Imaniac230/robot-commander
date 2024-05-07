from scipy.io.wavfile import write as write_wav
import nltk
import numpy as np
import sounddevice as sd
from dataclasses import dataclass
import threading as th
from typing_extensions import Self
from typing import List, Any

from bark import generate_audio, SAMPLE_RATE
from bark.generation import generate_text_semantic
from bark.api import semantic_to_waveform


@dataclass
class TTSParams:
    model_path: str
    voice: str


class TTS:
    def __init__(self, params: TTSParams) -> None:
        self.params: TTSParams = params
        self.last_generation: Any = None
        self.generated_file: str = "output_response.wav"
        self.server_worker: th.Thread | None = None
    
    def __del__(self):
        if self.server_worker is not None and self.server_worker.is_alive(): self.server_worker.join()

    def start_server(self) -> Self:
        if self.server_worker is not None and not self.server_worker.is_alive(): self.server_worker.start()
        return self
    
    def play_synthesis(self):
        if self.last_generation is not None:
            sd.play(self.last_generation, SAMPLE_RATE, blocking=True)


class Bark(TTS):
    def __init__(self, params: TTSParams) -> None:
        super().__init__(params)

    def start_server(self) -> bool:
        raise NotImplementedError

    def synthesize_short(self, short_prompt: str) -> Any:
        self.last_generation = generate_audio(short_prompt, history_prompt=self.params.voice,
                                              model_path=self.params.model_path)
        write_wav(self.generated_file, SAMPLE_RATE, self.last_generation)

        return self.last_generation

    def synthesize(self, prompt: str) -> Any:
        processed_prompt = nltk.sent_tokenize(prompt.replace('\n', ' ').strip())
        silence = np.zeros(int(0.25 * SAMPLE_RATE))

        pieces = []
        for section in processed_prompt:
            # audio_array = generate_audio(section, history_prompt=self.speaker_voice, model_path=self.model_path)
            # prevent audio hallucinations at section ends
            audio_array = semantic_to_waveform(
                generate_text_semantic(section, history_prompt=self.params.voice, temp=0.6, min_eos_p=0.05,
                                       model_path=self.params.model_path),
                history_prompt=self.params.voice,
                model_path=self.params.model_path)

            pieces += [audio_array, silence.copy()]
        self.last_generation = np.concatenate(pieces)
        write_wav(self.generated_file, SAMPLE_RATE, self.last_generation)

        return self.last_generation


class BarkCPP(TTS):
    def __init__(self, params: TTSParams) -> None:
        super().__init__(params)

    def start_server(self) -> bool:
        raise NotImplementedError("implement bark_cpp")

    def synthesize(self, prompt: str):
        raise NotImplementedError("implement bark_cpp")
