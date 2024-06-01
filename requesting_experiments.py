from utils import Requestor, Recorder
from pynput.keyboard import Key
import sounddevice as sd
import soundfile as sf
import argparse
import json


def general_requesting(use_local: bool) -> None:
    recording = "test.wav"
    print("\nPress and hold 'F10' to record your command ...\n")
    with Recorder(frequency_hz=16000) as r:
        r.hold_to_record(Key.f10)
        r.save_recording(recording)
    print(f"\nDone, created file: {recording}")

    print("\nPosting translation ...")
    if use_local:
        result = Requestor("http://" + local_address + ":8080").transcribe("/inference", {"file": (recording, open(recording, mode="rb"), "audio/x-wav")})
    else:
        result = Requestor("https://api.openai.com", api_key=key).transcribe("/v1/audio/translations", {"file": (recording, open(recording, mode="rb"), "audio/x-wav")}, {"model": "whisper-1"})
    if result is not None: print(f"{result['text']}")

    print("\nPosting chat ...")
    if use_local:
        result = Requestor("http://" + local_address + ":8081").respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}]})
    else:
        result = Requestor("https://api.openai.com", api_key=key).respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}], "model": "gpt-4o"})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    #TODO(tts-server): integrate with local bark_cpp when ready
    print("\nPosting audio ...")
    result = Requestor("https://api.openai.com", api_key=key).synthesize("/v1/audio/speech", {"model": "tts-1", "input": "what is an omelette?", "voice": "fable", "response_format": "wav"})
    if result is not None:
        with open(recording, mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read(recording)
        sd.play(data, rate, blocking=True)


def local_ros(prompt: str) -> None:
    print(f"\n\ttesting ROS with '{prompt}':")
    req = Requestor("http://" + local_address + ":8081")

    # print("\ncompletion:")
    # result = req.respond("/completion", {"prompt": "REQUEST:\n" + prompt, "stop": ["REQUEST:"], "json_schema": json.load(open("grammars/posestamped.json"))})
    # print(f'{result["content"]}')
    # print(f'{result}')

    #both seem to be giving good results
    print("\nchat-completion:")
    result = req.respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "REQUEST:\n" + prompt}], "stop": ["REQUEST:"], "response_format": {"type": "json_object", "schema": json.load(open("grammars/posestamped.json"))}})
    print(f"{result['choices'][0]['message']['content']}")
    # print(f'{result}')


def local_chat(prompt: str) -> None:
    print(f"\n\ttesting CHAT with '{prompt}':")
    req = Requestor("http://" + local_address + ":8083")

    # print("\ncompletion:")
    # result = req.respond("/completion", {"prompt": "REQUEST:\n" + prompt, "stop": ["REQUEST:"]})
    # print(f'{result["content"]}')
    # print(f'{result}')

    #using instruction fine-tuned seems to be giving best results
    print("\nchat-completion:")
    result = req.respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "REQUEST:\n" + prompt}], "stop": ["REQUEST:"]})
    print(f"{result['choices'][0]['message']['content']}")
    # print(f'{result}')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--local_address', type=str, default='127.0.0.1', help='Address for local server requests.')
    args: argparse.Namespace = parser.parse_args()

    key: str = args.key
    local_address: str = args.local_address

    general_requesting(use_local=True)
    p = "I want you to get your ass to the box."
    # local_ros(p)
    # local_chat(p)
