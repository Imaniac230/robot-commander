from utils import Requestor
import sounddevice as sd
import soundfile as sf
import argparse
import json


def general_requesting(use_local: bool) -> None:
    input_recording = "test_input.wav"
    output_audio = "test_output.wav"

    print("\nPosting translation ...")
    if use_local:
        result = Requestor("http://" + local_address + ":8080").transcribe("/inference", {"file": (input_recording, open(input_recording, mode="rb"), "audio/x-wav")})
    else:
        result = Requestor("https://api.openai.com", api_key=key).transcribe("/v1/audio/translations", {"file": (input_recording, open(input_recording, mode="rb"), "audio/x-wav")}, {"model": "whisper-1"})
    if result is not None: print(f"{result['text']}")

    print("\nPosting chat ...")
    if use_local:
        result = Requestor("http://" + local_address + ":8081").respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}]})
    else:
        result = Requestor("https://api.openai.com", api_key=key).respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}], "model": "gpt-4o"})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    #TODO(tts-server): using a custom finalization of the bark.cpp server,
    #   refactor once a stable release is available
    print("\nPosting audio ...")
    if use_local:
        result = Requestor("http://" + local_address + ":8082").synthesize("/v1/audio/speech", {"input": "what is an omelette?"})
    else:
        result = Requestor("https://api.openai.com", api_key=key).synthesize("/v1/audio/speech", {"input": "what is an omelette?", "voice": "fable", "response_format": "wav", "model": "tts-1"})
    if result is not None:
        with open(output_audio, mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read(output_audio)
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
    parser.add_argument('--use_local', action='store_true', default=False, help='Use local servers for requests. If false, will use OpenAI servers.')
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--local_address', type=str, default='127.0.0.1', help='Address for local server requests.')
    args: argparse.Namespace = parser.parse_args()

    key: str = args.key
    local_address: str = args.local_address

    general_requesting(args.use_local)
    # p = "I want you to get your ass to the box."
    # local_ros(p)
    # local_chat(p)
