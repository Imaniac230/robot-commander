from utils import Recorder
from ai_interface import LLAMACPP, WhisperCPP, Bark
from pynput.keyboard import Key

import argparse
import time
import roslibpy
import os
import glob
import json


def experiment() -> None:
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


def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--type', type=str, required=True, help='Message type for the topic.')

    args = parser.parse_args()
    return args


def main() -> None:
    args = args_factory()

    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()

    context = ["Window={position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}"]
    context += ["Door={position: {x: 10.0, y: -5.0, z: 0.0}, orientation: {w: 0.5}}"]
    context += ["Tomas={position: {x: -12.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}"]
    context += ["Tomas window={position: {x: 11.5, y: -4.2, z: 0.0}, orientation: {w: 0.5}}"]

    input_recording: str = "input_recording.wav"
    with open("/home/user/repos/foreign/llama_cpp/prompts/ros.txt") as f:
        init_prompt = f.read()

    messages = ""
    for file in glob.glob("messages/*.txt"):
        messages += "<" + os.path.basename(file).split('.')[0] + ">"
        with open(file, 'r') as rd:
            messages += rd.read()
        messages += "</" + os.path.basename(file).split('.')[0] + ">" + "\n"

    ctx = ""
    for item in context:
        ctx += item + "\n"

    if os.getenv("DEBUG") is not None:
        if ctx is not None:
            print(f'context:\n{ctx}')
        print(f'messages:\n{messages}')

    init_prompt += f'''\nFollowing lines specify the format of the required messages with their name in tags <name></name>:\n{messages}\n'''
    if ctx is not None:
        init_prompt += f'''Following lines provide context for mapping any position and orientation coordinates to keywords:\n{ctx}\n'''

    llama = LLAMACPP("/media/user/data_ssd/models/llama2/llama-2-13b/ggml-model-q4_0.gguf", init_prompt)
    whisper = WhisperCPP("/media/user/data_ssd/models/whisper/large/ggml-model-large-v3.bin")

    while True:
        try:
            print(f"\nPress and hold 'space' to record your command ...\n")
            # FIXME(recording): make it inline instead of this crappy file saving
            with Recorder(frequency_hz=16000) as r:
                r.hold_to_record(Key.space)
                r.save_recording(input_recording)
            print(f"\nTranscribing ...")
            prompt = whisper.transcribe(input_recording)
            print(f'Got prompt: "{prompt}"')
            print("Breaking down the goal and creating steps ...")
            messages = json.loads(llama.respond(f"<prompt>{prompt}<prompt>"))
            print("Done")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break

    ros_client.terminate()


if __name__ == '__main__':
    # experiment()
    main()
