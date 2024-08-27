from scipy.io.wavfile import write as write_wav
from dataclasses import dataclass
from typing_extensions import Self
from typing import List, Any, Optional, Dict

from bark import generate_audio, SAMPLE_RATE
from bark.generation import generate_text_semantic
from bark.api import semantic_to_waveform

from robot_commander_library.utils import Requestor

import sounddevice as sd
import soundfile as sf
import numpy as np
import subprocess as sp
import nltk
import os


@dataclass
class TTSParams:
    model_path: str
    voice: str
    server_hostname: Optional[str] = None
    server_port: Optional[int] = None
    n_of_threads_to_use: Optional[int] = None


class TTS:
    def __init__(self, params: TTSParams) -> None:
        self.params: TTSParams = params
        self.last_generation: Any = None
        self.generated_file: str = "output_response.wav"
        self.server_worker: Optional[sp.Popen] = None
        self.server_task: Optional[Any] = None

    def __del__(self) -> None:
        self.stop_server()

    def start_server(self) -> Self:
        if self.server_task is not None and not self.server_running(): self.server_worker = self.server_task()
        return self

    def server_running(self) -> bool:
        return self.server_worker is not None and self.server_worker.poll() is None

    def stop_server(self) -> Self:
        if self.server_running():
            self.server_worker.terminate()
            self.server_worker.wait()
            self.server_worker = None
        return self

    def synthesize(self, prompt: str, *args, **kwargs) -> Any: pass
    
    def play_synthesis(self):
        if self.last_generation is not None:
            sd.wait()
            sd.play(self.last_generation, SAMPLE_RATE, blocking=False)


class Bark(TTS):
    def __init__(self, params: TTSParams) -> None:
        super().__init__(params)

    def synthesize_short(self, short_prompt: str) -> Any:
        self.last_generation = generate_audio(short_prompt, history_prompt=self.params.voice,
                                              model_path=self.params.model_path)
        write_wav(self.generated_file, SAMPLE_RATE, self.last_generation)

        return self.last_generation

    def synthesize(self, prompt: str, load_model: bool = True) -> Any:
        if not load_model: print("Server for pytorch Bark is not supported, loading model.")

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


#TODO(bark-cpp): this is only a preliminary implementation to enable testing with the current bark.cpp state
#   refactor this once a stable release is available
class BarkCPP(TTS):
    def __init__(self, params: TTSParams) -> None:
        super().__init__(params)
        #TODO(bin-path): decide if we want to support installed bins as well
        self.library_path: str = os.getenv("ROBOT_COMMANDER_BARK_CPP_PATH", "")
        if not self.library_path: raise EnvironmentError("Required variable ROBOT_COMMANDER_BARK_CPP_PATH was not found.")
        self.bin_path: str = "build/examples"
        self.server_task = lambda : sp.Popen(self._build_command("server"))

    def _build_command(self, command: str, prompt: str = "") -> List[str]:
        full_command: str = self.library_path + '/' + self.bin_path + '/' + command + '/' + command

        # implicit params
        # configurable params (required)
        args: List[str] = ["--model", self.params.model_path]
        # configurable params (optional, default by bark.cpp implementation)
        if self.params.n_of_threads_to_use is not None: args += ["--threads", str(self.params.n_of_threads_to_use)]

        if command == "server":
            if self.params.server_hostname is not None: args += ["--address", self.params.server_hostname]
            if self.params.server_port is not None: args += ["--port", str(self.params.server_port)]
        elif command == "main":
            args += ["--prompt", prompt, "--outwav", self.generated_file]
        else:
            raise NotImplementedError(f"command '{command}' is not supported")

        return [full_command] + args

    def synthesize(self, prompt: str, load_model: bool = False) -> Any:
        #NOTE: the example main always stores the output into a wav file
        if load_model: sp.check_output(self._build_command("main", prompt))
        else:
            #TODO(failed-server): decide how to react if server is not alive at this point
            if self.server_running():
                #TODO(bark-cpp): using a custom finalization of the server, refactor this once there is a stable bark.cpp release
                payload = {"input": prompt}
                r: Optional[Dict] = Requestor("http://" + self.params.server_hostname + ':' + str(self.params.server_port)).synthesize("/v1/audio/speech", payload)
                if r is not None:
                    with open(self.generated_file, mode="wb") as f:
                        for data in r: f.write(data)

        self.last_generation = sf.read(self.generated_file)
        return self.last_generation
