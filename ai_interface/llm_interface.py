import subprocess as sp
import os


class LLAMACPP:
    def __init__(self, model_path: str, initial_prompt: str = "") -> None:
        self.model_path: str = model_path
        self.initial_prompt: str = initial_prompt
        self.library_path: str = "/home/user/repos/foreign/llama_cpp"
        self.bin_path: str = "build/bin"
        self.executable: str = "main"

        self.last_response: str = ""

    def respond(self, prompt: str, inline_response: bool = False) -> str:
        # naive call using the llama.cpp C++ main binary and loading the model each time
        '''
        ./build/bin/main -m /media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q8_0.gguf -n 2048 -c 2048
        -t 8 -tb 16 --color -r "User:" -f prompts/chat-with-bob.txt
        '''
        # args: str = " -n 2048 -c 2048 -t 8 -tb 16 --color -r \"User:\" -f prompts/chat-with-bob.txt "
        args: str = " -n 1024 -c 2048 -t 8 -tb 16 "  # -n 30 is good for short and relatively responses

        full_prompt: str = prompt if not self.initial_prompt else self.initial_prompt + ' ' + prompt
        command: str = self.library_path + '/' + self.bin_path + '/' + self.executable
        args += " --keep " + str(len(self.initial_prompt)) + " --model " + self.model_path + " --prompt \"" + full_prompt + "\""
        self.last_response = sp.check_output(command + args, shell=True, text=True)[len(full_prompt):]

        if os.getenv('DEBUG'):
            print(f"given prompt: {full_prompt}")
            print(f"llama response: {self.last_response}")

        if inline_response:
            self.last_response = self.last_response.replace('\n', ' ').strip()

        return self.last_response
