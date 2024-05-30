from dataclasses import dataclass
from typing_extensions import Self
from typing import List, Optional

from utils import Requestor

import subprocess as sp
import threading as th
import os


@dataclass
class STTParams:
    model_path: str
    initial_prompt: str
    server_hostname: Optional[str] = None
    server_port: Optional[int] = None
    n_of_threads_to_use: Optional[int] = None


class STT:
    def __init__(self, params: STTParams) -> None:
        self.params: STTParams = params
        self.last_transcription: str = ""
        self.server_worker: Optional[th.Thread] = None

    def __del__(self):
        if self.server_worker is not None and self.server_worker.is_alive(): self.server_worker.join()

    def start_server(self) -> Self:
        if self.server_worker is not None and not self.server_worker.is_alive(): self.server_worker.start()
        return self

    def transcribe(self, audio_file: str, *args, **kwargs) -> str: pass


class WhisperCPP(STT):
    # This should only be a minimal wrapper for the whisper.cpp project
    # and its examples, so that we can always support the latest versions.
    # We should not have to maintain any custom implementations here.
    # For a custom implementation, see https://github.com/mgonzs13/whisper_ros
    # with a more native integration of the whisper.cpp library functions into ROS.
    # We shouldn't attempt to do anything like that here.
    def __init__(self, params: STTParams) -> None:
        super().__init__(params)
        # TODO(paths): find a more canonical way of handling this (env vars?, installation?)
        self.library_path: str = str(os.path.realpath(__package__).rstrip(os.path.basename(__package__))) + 'libs/whisper_cpp'
        self.bin_path: str = "build/bin"
        # TODO: check failures
        self.server_worker = th.Thread(target=sp.run, args=[self._build_command("server")])

    def _build_command(self, command: str, file: str = "") -> List[str]:
        full_command: str = self.library_path + '/' + self.bin_path + '/' + command

        # implicit params
        # use translation mode if speaking in other than english
        args: List[str] = ["--translate", "--language", "auto"]
        # configurable params (required)
        args += ["--model", self.params.model_path]
        # this could be optional instead of passing an empty string
        args += ["--prompt", self.params.initial_prompt]
        # configurable params (optional, default by whisper.cpp implementation)
        if self.params.n_of_threads_to_use is not None: args += ["--threads", str(self.params.n_of_threads_to_use)]

        if command == "server":
            if self.params.server_hostname is not None: args += ["--host", self.params.server_hostname]
            if self.params.server_port is not None: args += ["--port", str(self.params.server_port)]
        elif command == "main":
            args += ["--file", file]
        else:
            raise NotImplementedError(f"command '{command}' is not supported")

        return [full_command] + args

    def transcribe(self, audio_file: str, load_model: bool = False) -> str:
        if load_model:
            raw_output: List[str] = sp.check_output(self._build_command("main", audio_file), text=True).split('\n')
            stripped_raw = []
            for line in raw_output:
                if len(line) > 0:
                    stripped_raw.append(line)

            if int(os.getenv("DEBUG", "0")) >= 2:
                print("\nstt full output:")
                for line in stripped_raw: print(f"{line}")
                print("\n")

            self.last_transcription = ''.join([line.split('] ')[1].strip() for line in stripped_raw])
        else:
            if self.server_worker.is_alive():
                #TODO(whisper_cpp): server endpoint is not fully oai compatible
                Requestor("http://" + self.params.server_hostname + ':' + str(self.params.server_port)).transcribe("/inference", audio_file)

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\nreturned stt transcription:\n{self.last_transcription}\n")

        return self.last_transcription
