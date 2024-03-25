from utils import Recorder
from ai_interface import LLAMACPP, WhisperCPP, Bark
from pynput.keyboard import Key


def experiment():
    input_recording: str = "input_recording.wav"
    with open("/home/user/repos/foreign/llama_cpp/prompts/chat.txt") as f:
        initial_prompt = f.read()

    llama = LLAMACPP("/media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q4_0.gguf", initial_prompt)
    whisper = WhisperCPP("/media/user/data_ssd/models/whisper/large/ggml-model-large-v3.bin")
    bark = Bark("/media/user/data_ssd/models/bark/")

    print(f"\nPress and hold 'space' to record your prompt ...")
    # FIXME(recording): make it inline instead of this crappy file saving
    with Recorder(frequency_hz=16000) as r:
        r.hold_to_record(Key.space)
        r.save_recording(input_recording)
    print(f"\nDone ...")

    bark.synthesize(llama.respond(whisper.transcribe(input_recording), inline_response=True))

    print(f"\nPlaying back the response ...")
    bark.play_synthesis()

    print(f"\nsummary:\n\t{input_recording} -> (whisper)\n\t-> '{whisper.last_transcription}' -> (llama)\n\t-> '{llama.last_response}' -> (bark)\n\t-> {bark.generated_file}")


if __name__ == '__main__':
    experiment()
