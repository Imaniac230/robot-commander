from dataclasses import dataclass
from typing_extensions import Self
from typing import List, Optional, Dict, Any

from robot_commander_library.utils import Requestor

import subprocess as sp
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
        #TODO(bin-path): decide if we want to support installed bins as well
        self.library_path: str = os.getenv("ROBOT_COMMANDER_WHISPER_CPP_PATH", "")
        if not self.library_path: raise EnvironmentError("Required variable ROBOT_COMMANDER_WHISPER_CPP_PATH was not found.")
        self.bin_path: str = "build/bin"
        self.server_task = lambda : sp.Popen(self._build_command("server"))

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
            #TODO(failed-server): decide how to react if server is not alive at this point
            if self.server_running():
                payload = {"file": (audio_file, open(audio_file, mode="rb"), "audio/x-wav")}
                #TODO(whisper_cpp): server does not have an oai compatible endpoint name
                r: Optional[Dict] = Requestor("http://" + self.params.server_hostname + ':' + str(self.params.server_port)).transcribe("/inference", payload)
                if r is not None:
                    self.last_transcription = r["text"]

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\nreturned stt transcription:\n{self.last_transcription}\n")

        return self.last_transcription
