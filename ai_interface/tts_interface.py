from scipy.io.wavfile import write as write_wav
import nltk
import numpy as np
import sounddevice as sd
from dataclasses import dataclass
import threading as th
from typing_extensions import Self

from bark import generate_audio, SAMPLE_RATE
from bark.generation import generate_text_semantic
from bark.api import semantic_to_waveform


@dataclass
class TTSParams: pass


# TODO(interfaces): create a genralized interface class that would be inherited by the various specializations here
class TTS:
    def __init__(self) -> None:
        pass


class Bark(TTS):
    def __init__(self, model_path: str, voice: str = "speaker_0") -> None:
        super().__init__()
        self.model_path: str = model_path
        self.speaker_voice: str = voice
        self.generated_file: str = "output_response.wav"

        self.last_generation = None

    def start_server(self) -> bool:
        raise NotImplementedError

    def synthesize_short(self, short_prompt: str):
        self.last_generation = generate_audio(short_prompt, history_prompt=self.speaker_voice,
                                              model_path=self.model_path)
        write_wav(self.generated_file, SAMPLE_RATE, self.last_generation)

        return self.last_generation

    def synthesize(self, prompt: str):
        processed_prompt = nltk.sent_tokenize(prompt.replace('\n', ' ').strip())
        silence = np.zeros(int(0.25 * SAMPLE_RATE))

        pieces = []
        for section in processed_prompt:
            # audio_array = generate_audio(section, history_prompt=self.speaker_voice, model_path=self.model_path)
            # prevent audio hallucinations at section ends
            audio_array = semantic_to_waveform(
                generate_text_semantic(section, history_prompt=self.speaker_voice, temp=0.6, min_eos_p=0.05,
                                       model_path=self.model_path),
                history_prompt=self.speaker_voice,
                model_path=self.model_path)

            pieces += [audio_array, silence.copy()]
        self.last_generation = np.concatenate(pieces)
        write_wav(self.generated_file, SAMPLE_RATE, self.last_generation)

        return self.last_generation

    def play_synthesis(self):
        if self.last_generation is not None:
            sd.play(self.last_generation, SAMPLE_RATE, blocking=True)


class BarkCPP(TTS):
    def __init__(self, model_path: str) -> None:
        super().__init__()
        self.model_path: str = model_path
        self.speaker_voice: str = "v2/en_speaker_5"
        self.generated_file: str = "output_response.wav"
        self.last_generation = None

    def start_server(self) -> bool:
        raise NotImplementedError("implement bark_cpp")

    def synthesize(self, prompt: str):
        raise NotImplementedError("implement bark_cpp")

    def play_synthesis(self):
        if self.last_generation is not None:
            sd.play(self.last_generation, SAMPLE_RATE, blocking=True)
