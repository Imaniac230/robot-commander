from utils import Requestor, Recorder
import sounddevice as sd
import soundfile as sf
import argparse

from pynput.keyboard import Key


def requesting(key: str) -> None:
    # req = Requestor("https://api.openai.com", api_key=key)
    # req = Requestor("http://10.0.0.131:8080", api_key=key) # whisper
    req = Requestor("http://10.0.0.131:8081", api_key=key) # llama
    # req = Requestor("http://10.0.0.131:8082", api_key=key) # whisper
    # req = Requestor("http://10.0.0.131:8083", api_key=key) # llama

    recording = "test.wav"
    # print("\nPress and hold 'F10' to record your command ...\n")
    # with Recorder(frequency_hz=16000) as r:
    #     r.hold_to_record(Key.f10)
    #     r.save_recording(recording)
    # print(f"\nDone, created file: {recording}")

    # oai works
    print("\nPosting chat ...")
    result = req.respond("/v1/chat/completions", "what is an omelette?")
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    # oai works
    #TODO(tts-server): integrate with bark_cpp when ready
    # print("\nPosting audio ...")
    # result = req.synthesize("/v1/audio/speech", dict(prompt="what is an omelette?", voice="fable"))
    # if result is not None:
    #     with open(recording, mode="wb") as f:
    #         for data in result: f.write(data)
    #     data, rate = sf.read(recording)
    #     sd.play(data, rate, blocking=True)

    # oai and local works.
    # print("\nPosting translation ...")
    # result = req.transcribe("/v1/audio/translations", recording)
    # result = req.transcribe("/inference", recording)
    # if result is not None: print(f"{result['text']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')

    requesting(parser.parse_args().key)
