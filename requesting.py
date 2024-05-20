from utils import Requestor
import sounddevice as sd
import soundfile as sf
import argparse


def requesting(key: str) -> None:
    req = Requestor("https://api.openai.com", api_key=key)

    prompt = "what is an omelette?"

    # works
    print("Posting chat ...")
    result = req.post_chat(prompt)
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    # works
    print("Posting audio ...")
    result = req.post_audio(prompt, "fable")
    if result is not None:
        with open("test.wav", mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read("test.wav")
        sd.play(data, rate, blocking=True)

    print("Posting translation ...")
    result = req.post_translation("test.wav")
    if result is not None: print(f"{result['text']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')

    requesting(parser.parse_args().key)
