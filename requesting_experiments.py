from utils import Requestor
import sounddevice as sd
import soundfile as sf
import argparse
import json


def general_requesting() -> None:
    input_recording = "test_input.wav"
    output_audio = "test_output.wav"

    print("\nPosting translation ...")
    if args.use_local:
        result = Requestor("http://" + args.local_address + ":8080").transcribe("/inference", {"file": (input_recording, open(input_recording, mode="rb"), "audio/x-wav")})
    else:
        result = Requestor("https://api.openai.com", api_key=args.key).transcribe("/v1/audio/translations", {"file": (input_recording, open(input_recording, mode="rb"), "audio/x-wav")}, {"model": "whisper-1"})
    if result is not None: print(f"{result['text']}")

    print("\nPosting chat ...")
    if args.use_local:
        result = Requestor("http://" + args.local_address + ":8081").respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}]})
    else:
        result = Requestor("https://api.openai.com", api_key=args.key).respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}], "model": "gpt-4o"})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    #TODO(tts-server): using a custom finalization of the bark.cpp server,
    #   refactor once a stable release is available
    print("\nPosting audio ...")
    if args.use_local:
        result = Requestor("http://" + args.local_address + ":8082").synthesize("/v1/audio/speech", {"input": "what is an omelette?"})
    else:
        result = Requestor("https://api.openai.com", api_key=args.key).synthesize("/v1/audio/speech", {"input": "what is an omelette?", "voice": "fable", "response_format": "wav", "model": "tts-1"})
    if result is not None:
        with open(output_audio, mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read(output_audio)
        sd.play(data, rate, blocking=True)


def local_ros(prompt: str) -> None:
    print(f"\n\ttesting ROS with '{prompt}':")
    req = Requestor("http://" + args.local_address + ":8081")

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
    req = Requestor("http://" + args.local_address + ":8083")

    # print("\ncompletion:")
    # result = req.respond("/completion", {"prompt": "REQUEST:\n" + prompt, "stop": ["REQUEST:"]})
    # print(f'{result["content"]}')
    # print(f'{result}')

    #using instruction fine-tuned seems to be giving best results
    print("\nchat-completion:")
    result = req.respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "REQUEST:\n" + prompt}], "stop": ["REQUEST:"]})
    print(f"{result['choices'][0]['message']['content']}")
    # print(f'{result}')

def local_bark() -> None:
    from ai_interface import Bark, TTSParams
    # prompt: str = "[smirks] Ah, the box, you say? Well, I'm afraid it's not that simple. You see, I've misplaced it... or rather, I've cleverly hidden it from prying eyes. [winks]"
    # prompt: str = "[laughs] Ah, mortal, you want me to do your bidding, do you? Very well, I shall condescend to retrieve the door. But don't think for a moment that I'll be opening it for just anyone. [s"
    # prompt: str = "[laughs] Ah, mortal, you want me to do your bidding, do you?"
    prompt: str = "[laughs] Ah, mortal, you ask a simple question, but I shall weave a tapestry of deceit around it. An omelette, you see, is a magical dish, born from the tears of the gods themselves. It's a"

    b = Bark(TTSParams(model_path=args.pytorch_tts_model_path, voice=args.chat_voice))
    # b.synthesize(prompt, load_model=True)
    # data, rate = sf.read(b.generated_file)
    # sd.play(data, rate, blocking=True)

    result = Requestor("http://" + args.local_address + ":8082").synthesize("/v1/audio/speech", {"input": prompt})
    if result is not None:
        with open("server_" + b.generated_file, mode="wb") as f:
            for data in result: f.write(data)
        data, rate = sf.read("server_" + b.generated_file)
        sd.play(data, rate, blocking=True)

def anthropic_support() -> None:
    print("\nposting to anthropic ...")
    result = Requestor("https://api.anthropic.com", api_key=args.anthropic_key).respond("/v1/messages", {"messages": [{"role": "user", "content": "what is an omelette?"}], "model": "claude-3-5-sonnet-20240620", "max_tokens": 1024})
    if result is not None: print(f"{result['content'][0]['text']}")

    print("\nposting to openai ...")
    result = Requestor("https://api.openai.com", api_key=args.openai_key).respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}], "model": "gpt-4o"})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")

    print("\nposting to local server ...")
    result = Requestor("http://" + args.local_address + ":8081").respond("/v1/chat/completions", {"messages": [{"role": "user", "content": "what is an omelette?"}]})
    if result is not None: print(f"{result['choices'][0]['message']['content']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_local', action='store_true', default=False, help='Use local servers for requests. If false, will use OpenAI servers.')
    parser.add_argument('--key', type=str, default=None, help='API key.')
    parser.add_argument('--openai_key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--anthropic_key', type=str, default=None, help='Anthropic API key.')
    parser.add_argument('--local_address', type=str, default='127.0.0.1', help='Address for local server requests.')

    parser.add_argument('--chat_voice', type=str, default=None, help='Voice type to be used by the tts. Note that local SunoAI and remote OpenAI use different options. For local, this is only valid when using the pytorch models here.')
    parser.add_argument('--pytorch_tts_model_path', type=str, default=None, help='Path to the local pytorch tts model files. Use this for better results, the server is currently only experimental.')
    args: argparse.Namespace = parser.parse_args()

    # general_requesting()
    # p = "I want you to get your ass to the box."
    # local_ros(p)
    # local_chat(p)
    # local_bark()
    anthropic_support()
