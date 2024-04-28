import subprocess as sp
import os

#TODO(interfaces): create a genralized interface class that would be inherited by the various specializations here
class LLMInterface:
    def __init__(self) -> None:
        pass


class LlamaCPP(LLMInterface):
    # This should only be a minimal wrapper for the llama.cpp project
    # and its examples, so that we can always support the latest versions.
    # We should not have to maintain any custom implementations here.
    # For a custom implementation, see: https://github.com/mgonzs13/llama_ros
    # with a more native integration of the llama.cpp library functions into ROS.
    # We shouldn't attempt to do anything like that here.
    def __init__(self, model_path: str, initial_prompt: str = "") -> None:
        self.model_path: str = model_path
        self.initial_prompt: str = initial_prompt
        #TODO(paths): find a more canonical way of handling this (env vars?, installation?)
        self.library_path: str = os.path.realpath(__package__).rstrip(os.path.basename(__package__)) + 'libs/llama_cpp'
        self.bin_path: str = "build/bin"

        self.last_response: str = ""

    def start_server(self) -> bool:
        raise NotImplementedError

    def respond(self, prompt: str, inline_response: bool = False, load_model: bool = False) -> str:
        full_prompt: str = prompt if not self.initial_prompt else self.initial_prompt + ' ' + prompt

        #TODO(args): expose these as options to init
        args: str = " --n-predict 1024 --ctx-size 2048 --threads 16 --threads-batch 16 "
        grammars_file = os.path.realpath(__package__).rstrip(os.path.basename(__package__)) + 'grammars/posestamped.gbnf'
        args += "--grammar-file " + grammars_file + " "
        args += "--n-gpu-layers 43 "
        args += " --keep " + str(len(self.initial_prompt)) + " --model " + self.model_path + " --prompt \"" + full_prompt + "\""

        if load_model:
            '''
            ./build/bin/main -m /media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q8_0.gguf -n 2048 -c 2048
            -t 16 -tb 16 --color -r "User:" -f prompts/chat-with-bob.txt
            '''
            command: str = self.library_path + '/' + self.bin_path + '/main'
            # this will not output the " character so if full_prompt contains any ", the slice will not be correct
            self.last_response = sp.check_output(command + args, shell=True, text=True)[len(full_prompt):]
        else:
            #TODO(server): implement a generalized OAI chat-completion interface to query a server,
            # that would be called from here
            # also check if the server is avialable at this point?
            raise NotImplementedError("implement query to server!'")

        if os.getenv('DEBUG'):
            print(f"given prompt: {full_prompt}")
            print(f"llama response: {self.last_response}")

        if inline_response:
            self.last_response = self.last_response.replace('\n', ' ').strip()

        return self.last_response
