import subprocess as sp
import os


class WhisperCPP:
    def __init__(self, model_path: str) -> None:
        self.model_path: str = model_path
        self.library_path: str = "/home/user/repos/foreign/whisper_cpp/"
        self.bin_path: str = "build/bin/"
        self.last_transcription: str = ""

    def transcribe(self, audio_file: str) -> str:
        # naive call using the whisper.cpp C++ main binary
        '''
        ./build/bin/main -m /media/user/data_ssd/models/whisper/small/ggml-model-small.bin --print-colors -f prompt.wav
        '''
        # args: str = " --print-colors "
        args: str = ""

        # a = sp.run(executable=whisper_path + cmd, args=" -m " + model + " -f " + audio_file + args, stdout=sp.PIPE)
        # a = sp.run(whisper_path + cmd)
        # print(f"\n\ntype: {type(a)}, data: {a}\n\n")
        # os.system(whisper_path + cmd + " -m " + model + " -f " + audio_file + args)
        raw_output = sp.check_output(
            self.library_path + self.bin_path + "main -m " + self.model_path + " -f " + audio_file + args,
            shell=True, text=True).split('\n')
        stripped_raw = []
        for line in raw_output:
            if len(line) > 0:
                stripped_raw.append(line)

        if os.getenv('DEBUG'):
            print(f"\nrecorded transcription:")
            for line in stripped_raw: print(f"{line}")

        self.last_transcription = ''.join([line.split('] ')[1].strip() for line in stripped_raw])

        return self.last_transcription
