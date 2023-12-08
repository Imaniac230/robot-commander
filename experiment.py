import utils
from ai_interface import LLAMAInterface, WhisperInterface, BarkInterface

def experiment():
    input_recording: str = "input_recording.wav"

    llama = LLAMAInterface("/media/user/data_ssd/models/llama2/llama-2-13b-chat/ggml-model-q8_0.gguf")
    whisper = WhisperInterface("/media/user/data_ssd/models/whisper/small/ggml-model-small.bin")
    bark = BarkInterface("/media/user/data_ssd/models/bark/")

    print(f"Recording your prompt ...")
    utils.record_audio(5, input_recording, 16000)
    print(f"Done ...")

    bark.synthesize(llama.respond(whisper.transcribe(input_recording)))

    print(f"Playing back the response ...")
    bark.play_synthesis()

    print(f"\nsummary:\n{input_recording} (whisper)-> '{whisper.last_transcription}' (llama)-> '{llama.last_response}' (bark)-> {bark.generated_file}")


if __name__ == '__main__':
    experiment()