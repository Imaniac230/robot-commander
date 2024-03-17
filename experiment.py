from utils import Recorder
from ai_interface import LLAMA, Whisper, Bark
from pynput.keyboard import Key


def experiment():
    input_recording: str = "input_recording.wav"

    llama = LLAMA("/media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q8_0.gguf")
    whisper = Whisper("/media/user/data_ssd/models/whisper/small/ggml-model-small.bin")
    bark = Bark("/media/user/data_ssd/models/bark/")

    print(f"Press and hold 'space' to record your prompt ...")
    # FIXME(recording): make it inline instead of this crappy file saving
    with Recorder(frequency_hz=16000) as r:
        r.hold_to_record(Key.space)
        r.save_recording(input_recording)
    print(f"Done ...")

    bark.synthesize(llama.respond(whisper.transcribe(input_recording)))

    print(f"Playing back the response ...")
    bark.play_synthesis()

    print(
        f"\nsummary:\n{input_recording} (whisper)-> '{whisper.last_transcription}' (llama)-> '{llama.last_response}' "
        f"(bark)-> {bark.generated_file}")


if __name__ == '__main__':
    experiment()
