from utils import Requestor
import sounddevice as sd
import soundfile as sf
import argparse


def requesting(key: str) -> None:
    req = Requestor("https://api.openai.com", api_key=key)
    # req = Requestor("http://127.0.1.1:8080", api_key=key) # whisper
    # req = Requestor("http://127.0.1.1:8081", api_key=key) # llama
    # req = Requestor("http://127.0.1.1:8082", api_key=key) # whisper
    # req = Requestor("http://127.0.1.1:8083", api_key=key) # llama

    prompt = "what is an omelette?"

    # oai works
    print("\nPosting chat ...")
    result = req.respond(prompt)
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    # oai works
    print("\nPosting audio ...")
    result = req.synthesize(prompt, "fable")
    if result is not None:
        with open("test.wav", mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read("test.wav")
        sd.play(data, rate, blocking=True)

    # oai works.
    print("\nPosting translation ...")
    result = req.transcribe("test.wav")
    if result is not None: print(f"{result['text']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')

    requesting(parser.parse_args().key)
