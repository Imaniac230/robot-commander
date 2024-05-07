import subprocess as sp
import os
from dataclasses import dataclass
import threading as th
from typing_extensions import Self
from typing import List


@dataclass
class LLMParams:
    model_path: str
    initial_prompt: str
    n_of_tokens_to_predict: int
    temperature: float = None
    n_of_gpu_layers_to_offload: int = None
    grammar_file_path: str = None
    server_hostname: str = None
    server_port: int = None
    n_of_parallel_server_requests: int = None
    n_of_threads_to_use: int = None


class LLM:
    def __init__(self, params: LLMParams) -> None:
        self.params: LLMParams = params
        self.last_response: str = ""
        self.server_worker: th.Thread | None = None

    def __del__(self):
        if self.server_worker is not None and self.server_worker.is_alive(): self.server_worker.join()

    def start_server(self) -> Self:
        if self.server_worker is not None and not self.server_worker.is_alive(): self.server_worker.start()
        return self


class LlamaCPP(LLM):
    # This should only be a minimal wrapper for the llama.cpp project
    # and its examples, so that we can always support the latest versions.
    # We should not have to maintain any custom implementations here.
    # For a custom implementation, see: https://github.com/mgonzs13/llama_ros
    # with a more native integration of the llama.cpp library functions into ROS.
    # We shouldn't attempt to do anything like that here.
    def __init__(self, params: LLMParams) -> None:
        super().__init__(params)
        # TODO(paths): find a more canonical way of handling this (env vars?, installation?)
        self.library_path: str = str(os.path.realpath(__package__).rstrip(os.path.basename(__package__))) + 'libs/llama_cpp'
        self.bin_path: str = "build/bin"
        # TODO: check failures
        self.server_worker = th.Thread(target=sp.run, args=[self._build_command("server")])

    def _build_command(self, command: str, prompt: str = "") -> List[str]:
        full_command: str = self.library_path + '/' + self.bin_path + '/' + command

        # implicit params
        # load context size from model hparams
        args: List[str] = ["--ctx-size", str(0)]
        # configurable params (required)
        args += ["--model", self.params.model_path]
        args += ["--n-predict", str(self.params.n_of_tokens_to_predict)]
        # configurable params (optional, default by llama.cpp implementation)
        if self.params.n_of_gpu_layers_to_offload is not None: args += ["--n-gpu-layers", str(self.params.n_of_gpu_layers_to_offload)]
        if self.params.n_of_threads_to_use is not None: args += ["--threads", str(self.params.n_of_threads_to_use), "--threads-batch", str(self.params.n_of_threads_to_use)]

        if command == "server":
            # implicit params
            # publish server metrics
            args += ["--metrics"]
            # configurable params (optional, default by llama.cpp implementation)
            if self.params.n_of_parallel_server_requests is not None: args += ["--parallel", str(self.params.n_of_parallel_server_requests)]
            if self.params.server_hostname is not None: args += ["--host", self.params.server_hostname]
            if self.params.server_port is not None: args += ["--port", str(self.params.server_port)]
            # [grammar, temperature] are specified in each server request
            # initial prompt is specified in a json config to server at launch
        elif command == "main":
            # implicit params
            # don't output the prompt
            args += ["--no-display-prompt"]
            # we can't provide both an input file and a prompt, so we combine the initial prompt with the provided prompt here
            args += ["--keep", str(len(self.params.initial_prompt)), "--prompt", "\"" + prompt + "\""]
            # configurable params (optional, default by llama.cpp implementation)
            if self.params.temperature is not None: args += ["--temp", str(self.params.temperature)]
            if self.params.grammar_file_path is not None: args += ["--grammar-file", self.params.grammar_file_path]
        else:
            raise NotImplementedError(f"command '{command}' is not supported")

        return [full_command] + args

    def respond(self, prompt: str, inline_response: bool = False, load_model: bool = False) -> str:
        full_prompt: str = prompt if not self.params.initial_prompt else self.params.initial_prompt + ' ' + prompt

        if load_model:
            self.last_response = sp.check_output(self._build_command("main", full_prompt), text=True)

            if int(os.getenv("DEBUG", "0")) >= 2:
                print(f"\nllm command full output:\n{self.last_response}\n")
        else:
            if self.server_worker.is_alive():
                # TODO(server): implement a generalized OAI chat-completion interface to query a server,
                # that would be called from here
                # also check if the server is avialable at this point?
                raise NotImplementedError("implement query to server!'")

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven prompt:\n{full_prompt}\n")
            print(f"\nreturned llm response:\n{self.last_response}\n")

        if inline_response:
            self.last_response = self.last_response.replace('\n', ' ').strip()

        return self.last_response
