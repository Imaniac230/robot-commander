import json

from utils import Requestor, Recorder
import sounddevice as sd
import soundfile as sf
import argparse

from pynput.keyboard import Key


def requesting(key: str) -> None:
    # req = Requestor("https://api.openai.com", api_key=key)
    # req = Requestor("http://10.0.0.131:8080", api_key=key) # whisper
    # req = Requestor("http://10.0.0.131:8081", api_key=key) # llama ros
    # req = Requestor("http://10.0.0.131:8082", api_key=key) # whisper
    req = Requestor("http://10.0.0.131:8083", api_key=key) # llama chat

    recording = "test.wav"
    print("\nPress and hold 'F10' to record your command ...\n")
    with Recorder(frequency_hz=16000) as r:
        r.hold_to_record(Key.f10)
        r.save_recording(recording)
    print(f"\nDone, created file: {recording}")

    # oai works
    print("\nPosting chat ...")
    result = req.respond("/v1/chat/completions", {"model": "gpt-4o", "messages": [{"role": "user", "content": "go to the door"}]})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    # oai works
    #TODO(tts-server): integrate with bark_cpp when ready
    print("\nPosting audio ...")
    result = req.synthesize("/v1/audio/speech", {"model": "tts-1", "input": "what is an omelette?", "voice": "fable", "response_format": "wav"})
    if result is not None:
        with open(recording, mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read(recording)
        sd.play(data, rate, blocking=True)

    # oai and local works.
    print("\nPosting translation ...")
    # result = req.transcribe("/v1/audio/translations", {"file": (recording, open(recording, mode="rb"), "audio/x-wav")}, {"model": "whisper-1"})
    result = req.transcribe("/inference", {"file": (recording, open(recording, mode="rb"), "audio/x-wav")})
    if result is not None: print(f"{result['text']}")

def ros(prompt: str):
    print(f"\n\ttesting ROS with '{prompt}':")
    req = Requestor("http://10.0.0.131:8081") # llama ros

    # print("\ncompletion:")
    # result = req.respond("/completion", {"prompt": "REQUEST:\n" + prompt, "stop": ["REQUEST:"], "json_schema": json.load(open("grammars/posestamped.json"))})
    # print(f'{result["content"]}')
    # print(f'{result}')

    #both seem to be giving good results
    print("\nchat-completion:")
    result = req.respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "REQUEST:\n" + prompt}], "stop": ["REQUEST:"], "response_format": {"type": "json_object", "schema": json.load(open("grammars/posestamped.json"))}})
    print(f"{result['choices'][0]['message']['content']}")
    # print(f'{result}')

def chat(prompt: str):
    print(f"\n\ttesting CHAT with '{prompt}':")
    req = Requestor("http://10.0.0.131:8083") # llama chat

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

    # requesting(parser.parse_args().key)
    p = "I want you to get your ass to the box."
    ros(p)
    chat(p)
