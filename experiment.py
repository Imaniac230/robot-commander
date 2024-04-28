from utils import Recorder
from ai_interface import LlamaCPP, WhisperCPP, Bark
from pynput.keyboard import Key

import argparse
import time
import roslibpy
import os
import glob
import json


def conversation_agent(stt_model: str, llm_model: str, tts_model: str, tts_voice: str) -> None:
    input_recording: str = "input_recording.wav"
    with open("prompts/robot-chat.txt") as f:
        initial_prompt = f.read()

    llama = LlamaCPP(llm_model, initial_prompt)
    whisper = WhisperCPP(tts_model)
    bark = Bark(stt_model, tts_voice)

    print(f"\nPress and hold 'space' to record your prompt ...")
    # FIXME(recording): make it inline instead of this crappy file saving
    with Recorder(frequency_hz=16000) as r:
        r.hold_to_record(Key.space)
        r.save_recording(input_recording)
    print(f"\nDone ...")

    bark.synthesize(llama.respond(whisper.transcribe(input_recording, load_model=True), inline_response=True, load_model=True))

    print(f"\nPlaying back the response ...")
    bark.play_synthesis()

    print(f"\nsummary:\n\t{input_recording} -> (whisper)\n\t-> '{whisper.last_transcription}' -> (llama)\n\t-> '{llama.last_response}' -> (bark)\n\t-> {bark.generated_file}")


def ros_publisher_agent(stt_model: str, llm_model: str) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--type', type=str, required=True, help='Message type for the topic.')
    args = parser.parse_args()

    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()

    context = ['keyword: window -> frame_id: map, position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: door -> frame_id: map, position: {x: 10.0, y: -5.0, z: 0.0}, orientation: {w: 0.5}']
    context += ['keyword: Tomas -> frame_id: map, position: {x: -12.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Milos -> frame_id: map, position: {x: -1.0, y: -4.0.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Petr -> frame_id: map, position: {x: 15.0.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Tomas window -> frame_id: map, position: {x: 11.5, y: -4.2, z: 0.0}, orientation: {w: 0.5}']
    context += ['keyword: origin -> frame_id: map, position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}']

    input_recording: str = "recording.wav"
    with open("prompts/ros-publisher.txt") as f:
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

    init_prompt += 'Details about the ROS2 messages that you will output:\n'
    init_prompt += f'The following shows the ROS2 messages in the required JSON format:\n{messages}\n'
    init_prompt += 'Each message is enclosed inside a tag that defines its ROS2 name (<message-name></message-name>).\n\n'
    if ctx is not None:
        init_prompt += 'Additional properties of the environment that you are operating in:\n'
        init_prompt += f'The following lines provide context for mapping any received keywords to corresponding properties of the JSON response:\n{ctx}\n'
        init_prompt += 'Make sure you always match all JSON property values exactly each time you detect a corresponding keyword.\n\n'

    llama = LlamaCPP(llm_model, init_prompt)
    whisper = WhisperCPP(stt_model)

    while True:
        try:
            print(f"\nPress and hold 'space' to record your command ...\n")
            # FIXME(recording): make it inline instead of this crappy file saving
            with Recorder(frequency_hz=16000) as r:
                r.hold_to_record(Key.space)
                r.save_recording(input_recording)
            print(f"\nTranscribing ...")
            prompt = whisper.transcribe(input_recording, load_model=True)
            print(f'Got prompt: "{prompt}"')
            print("Breaking down the goal and creating steps ...")
            messages = json.loads(llama.respond(f"<prompt>{prompt}</prompt>", load_model=True))["commands"]
            print("Done")
            input("testing")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except:
            continue

    ros_client.terminate()


if __name__ == '__main__':
    llm: str = "/media/user/data_ssd/models/llama2/llama-2-13b/ggml-model-q4_0.gguf"
    stt: str = "/media/user/data_ssd/models/whisper/large/ggml-model-large-v3.bin"
    tts: str = "/media/user/data_ssd/models/bark/"
    tts_voice: str = "v2/en_speaker_5"

    conversation_agent(stt, llm, tts, tts_voice)
    # ros_publisher_agent(stt, llm)
