import subprocess as sp
import os


class LLAMA:
    def __init__(self, model_path: str) -> None:
        self.model_path: str = model_path
        self.library_path: str = "/home/user/repos/foreign/llama_cpp/"
        self.bin_path: str = "build/bin/"
        self.last_response: str = ""

    def respond(self, prompt: str) -> str:
        # naive call using the llama.cpp C++ main binary
        '''
        ./build/bin/main -m /media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q8_0.gguf -n 2048 -c 2048
        -t 8 -tb 16 --color -r "User:" -f prompts/chat-with-bob.txt
        '''
        # args: str = " -n 2048 -c 2048 -t 8 -tb 16 --color -r \"User:\" -f prompts/chat-with-bob.txt "
        args: str = " -n 30 -c 2048 -t 8 -tb 16 "  # -n 30 is good for short and relatively responses

        self.last_response = sp.check_output(
            self.library_path + self.bin_path + "main -m " + self.model_path + " --prompt \"" + prompt + "\"" + args,
            shell=True, text=True).replace(prompt, '')

        if os.getenv('DEBUG'):
            print(f"llama response: {self.last_response}")

        return self.last_response
