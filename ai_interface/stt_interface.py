import subprocess as sp
import os

#TODO(interfaces): create a genralized interface class that would be inherited by the various specializations here
class STTInterface:
    def __init__(self) -> None:
        pass


class WhisperCPP(STTInterface):
    # This should only be a minimal wrapper for the whisper.cpp project
    # and its examples, so that we can always support the latest versions.
    # We should not have to maintain any custom implementations here.
    # For a custom implementation, see https://github.com/mgonzs13/whisper_ros
    # with a more native integration of the whisper.cpp library functions into ROS.
    # We shouldn't attempt to do anything like that here.
    def __init__(self, model_path: str) -> None:
        self.model_path: str = model_path
        #TODO(paths): find a more canonical way of handling this (env vars?, installation?)
        self.library_path: str = os.path.realpath(__package__).rstrip(os.path.basename(__package__)) + 'libs/whisper_cpp'
        self.bin_path: str = "build/bin"

        self.last_transcription: str = ""

    def start_server(self) -> bool:
        raise NotImplementedError

    def transcribe(self, audio_file: str, load_model: bool = False) -> str:
        #TODO(args): expose these as options to init
        args: str = ""
        args += " --model " + self.model_path + " --file " + audio_file

        if load_model:
            '''
            ./build/bin/main -m /media/user/data_ssd/models/whisper/large/ggml-model-q4_0-large-v3.bin --print-colors -f prompt.wav
            '''
            command: str = self.library_path + '/' + self.bin_path + '/main'
            raw_output: list[str] = sp.check_output(command + args, shell=True, text=True).split('\n')
            stripped_raw = []
            for line in raw_output:
                if len(line) > 0:
                    stripped_raw.append(line)
            self.last_transcription = ''.join([line.split('] ')[1].strip() for line in stripped_raw])
        else:
            #TODO(server): implement a generalized OAI translate or transcribe interface to query a server,
            # that would be called from here
            # also check if the server is avialable at this point?
            raise NotImplementedError("implement query to server!'")

        if os.getenv('DEBUG'):
            print(f"\nrecorded transcription:")
            for line in stripped_raw: print(f"{line}")

        return self.last_transcription
