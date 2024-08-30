from dataclasses import dataclass
from typing_extensions import Self
from typing import List, Optional, Dict, Any

from robot_commander_library.utils import Requestor

import subprocess as sp
import os
import json


@dataclass
class LLMParams:
    model_path: str
    initial_prompt: str
    n_of_tokens_to_predict: int
    temperature: Optional[float] = None
    n_of_gpu_layers_to_offload: Optional[int] = None
    json_schema_file_path: Optional[str] = None
    grammar_file_path: Optional[str] = None #TODO(grammar): decide if we will actually still use this
    server_hostname: Optional[str] = None
    server_port: Optional[int] = None
    n_of_parallel_server_requests: Optional[int] = None
    n_of_threads_to_use: Optional[int] = None


class LLM:
    def __init__(self, params: LLMParams) -> None:
        self.params: LLMParams = params
        self.last_response: str = ""
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

    def respond(self, prompt: str, *args, **kwargs) -> str: pass


class LlamaCPP(LLM):
    # This should only be a minimal wrapper for the llama.cpp project
    # and its examples, so that we can always support the latest versions.
    # We should not have to maintain any custom implementations here.
    # For a custom implementation, see: https://github.com/mgonzs13/llama_ros
    # with a more native integration of the llama.cpp library functions into ROS.
    # We shouldn't attempt to do anything like that here.
    def __init__(self, params: LLMParams) -> None:
        super().__init__(params)
        self.server_task = lambda : sp.Popen(self._build_command("llama-server"))

    def _build_command(self, command: str, prompt: str = "") -> List[str]:
        # implicit params
        # load context size from model hparams
        args: List[str] = ["--ctx-size", str(0)]
        # configurable params (required)
        args += ["--model", self.params.model_path, "--n-predict", str(self.params.n_of_tokens_to_predict)]
        # configurable params (optional, default by llama.cpp implementation)
        if self.params.n_of_gpu_layers_to_offload is not None: args += ["--n-gpu-layers", str(self.params.n_of_gpu_layers_to_offload)]
        if self.params.n_of_threads_to_use is not None: args += ["--threads", str(self.params.n_of_threads_to_use), "--threads-batch", str(self.params.n_of_threads_to_use)]

        if command == "llama-server":
            # implicit params
            # publish server metrics
            args += ["--metrics"]
            # initial prompt must be specified in a file
            #TODO(system-prompt): server readme specifies a json formatted file for the system prompt,
            #   but the code seems to just pass the raw text contents as is without any json parsing
            # tmp = "/tmp/system_prompt.json"
            # with open(tmp, "w") as f: f.write(json.dumps({"system_prompt": {"prompt": self.params.initial_prompt, "anti_prompt": "Human:", "assistant_name": "Loki:"}}))
            #TODO(system-prompt): decide how to approach this:
            #   each request in chat-completion "system" role? -> usable only with instruction fine tuned models with chat templates?
            #   this system_prompt? -> gets tokenized directly at the start without the special tokens? - only usable with with foundation models?
            from random import randint
            tmp = "/tmp/system_prompt_" + str(randint(0, 1000)) + ".txt"
            with open(tmp, "w") as f: f.write(self.params.initial_prompt)
            args += ["--system-prompt-file", tmp]
            # configurable params (optional, default by llama.cpp implementation)
            if self.params.n_of_parallel_server_requests is not None: args += ["--parallel", str(self.params.n_of_parallel_server_requests)]
            if self.params.server_hostname is not None: args += ["--host", self.params.server_hostname]
            if self.params.server_port is not None: args += ["--port", str(self.params.server_port)]
            # [grammar, temperature, n_keep] are specified in each server request
        elif command == "llama-cli":
            # implicit params
            # don't output the given prompt and stop generation on specified sequence
            args += ["--no-display-prompt", "--reverse-prompt", "REQUEST:"]
            # we can't provide both an input file and a prompt, so we combine the initial prompt with the provided prompt here
            args += ["--keep", str(len(self.params.initial_prompt)), "--prompt", "\"" + prompt + "\""]
            # configurable params (optional, default by llama.cpp implementation)
            if self.params.temperature is not None: args += ["--temp", str(self.params.temperature)]
            #TODO(grammar): currently using only schema, decide if we ever need to specify the grammar
            # if self.params.grammar_file_path is not None: args += ["--grammar-file", self.params.grammar_file_path]
            if self.params.json_schema_file_path is not None:
                with open(self.params.json_schema_file_path, 'r') as sch: args += ["--json-schema", json.dumps(json.load(sch))]
        else:
            raise NotImplementedError(f"command '{command}' is not supported")

        return [command] + args

    def respond(self, prompt: str, inline_response: bool = False, load_model: bool = False) -> str:
        full_prompt: str = prompt if not self.params.initial_prompt else self.params.initial_prompt + 'REQUEST:\n' + prompt

        if load_model:
            self.last_response = sp.check_output(self._build_command("llama-cli", full_prompt), text=True)

            if int(os.getenv("DEBUG", "0")) >= 2:
                print(f"\nllm command full output:\n{self.last_response}\n")
        else:
            #TODO(failed-server): decide how to react if server is not alive at this point
            if self.server_running():
                payload = {"messages": [{"role": "user", "content": "REQUEST:\n" + prompt}], "stop": ["REQUEST:"]}
                #TODO(grammar): decide if we will handle both grammar and schema options
                if self.params.json_schema_file_path is not None:
                    with open(self.params.json_schema_file_path, 'r') as sch: payload["response_format"] = {"type": "json_object", "schema": json.load(sch)}

                r: Optional[Dict] = Requestor("http://" + self.params.server_hostname + ':' + str(self.params.server_port)).respond("/v1/chat/completions", payload)
                if r is not None:
                    self.last_response = r["choices"][0]["message"]["content"]

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven prompt:\n{full_prompt}\n")
            print(f"\nreturned llm response:\n{self.last_response}\n")

        if inline_response:
            self.last_response = self.last_response.replace('\n', ' ').strip()

        return self.last_response
