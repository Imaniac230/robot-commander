from typing import Tuple, Dict

import argparse
import time
import roslibpy
import glob
import os
import json


def ros_factory() -> Tuple[roslibpy.Ros, argparse.Namespace]:
    parser = argparse.ArgumentParser()
    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--type', type=str, required=True, help='Message type for the topic.')
    args: argparse.Namespace = parser.parse_args()

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()

    return ros, args


# TODO(prompt-factory): create a class for this
def prompt_factory() -> Dict[str, str]:
    context = ['keyword: window -> frame_id: map, position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: door -> frame_id: map, position: {x: 10.0, y: -5.0, z: 0.0}, orientation: {w: 0.5}']
    context += ['keyword: Tomas -> frame_id: map, position: {x: -12.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Milos -> frame_id: map, position: {x: -1.0, y: -4.0.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Petr -> frame_id: map, position: {x: 15.0.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}']
    context += ['keyword: Tomas window -> frame_id: map, position: {x: 11.5, y: -4.2, z: 0.0}, orientation: {w: 0.5}']
    context += ['keyword: origin -> frame_id: map, position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}']

    messages = ""
    for file in glob.glob("messages/*.txt"):
        messages += "<" + os.path.basename(file).split('.')[0] + ">"
        with open(file, 'r') as rd:
            messages += rd.read()
        messages += "</" + os.path.basename(file).split('.')[0] + ">" + "\n"

    ctx = ""
    for item in context:
        ctx += item + "\n"

    if int(os.getenv("DEBUG", "0")) >= 1:
        if ctx is not None:
            print(f"context:\n{ctx}")
        print(f"messages:\n{messages}")

    with open("prompts/ros-publisher.txt") as f: ros_agent_prompt = f.read()
    ros_agent_prompt += 'Details about the ROS2 messages that you will output:\n'
    ros_agent_prompt += f'The following shows the ROS2 messages in the required JSON format:\n{messages}\n'
    ros_agent_prompt += 'Each message is enclosed inside a tag that defines its ROS2 name (<message-name></message-name>).\n\n'
    if ctx is not None:
        ros_agent_prompt += 'Additional properties of the environment that you are operating in:\n'
        ros_agent_prompt += f'The following lines provide context for mapping any received keywords to corresponding properties of the JSON response:\n{ctx}\n'
        ros_agent_prompt += 'Make sure you always match all JSON property values exactly each time you detect a corresponding keyword.\n\n'

    with open("prompts/robot-chat.txt") as f: chat_agent_prompt = f.read()

    return dict(chat=chat_agent_prompt, ros=ros_agent_prompt)


def openai_example(system_init: Tuple[roslibpy.Ros, argparse.Namespace], prompt_init: Dict[str, str]) -> None:
    from ai_interface import OpenAI, OpenAIParams

    ros_client: roslibpy.Ros = system_init[0]
    args: argparse.Namespace = system_init[1]
    ros_agent = OpenAI(OpenAIParams(api_key=args.key, initial_chat_prompt=prompt_init["ros"]))
    chat_agent = OpenAI(OpenAIParams(api_key=args.key, initial_chat_prompt=prompt_init["chat"]))
    while True:
        try:
            print("\nPress and hold 'space' to record your command ...\n")
            prompt: str = chat_agent.get_voice_prompt()

            print("\nResponding ...")
            response: str = chat_agent.get_messages(prompt)
            print("\nDone")

            print(f"\nsummary:\n\t"
                  f"(whisper)\n\t-> '{prompt}' -> "
                  f"(chatgpt)\n\t-> '{response}'")

            print("\nGenerating commands ...")
            messages = json.loads(ros_agent.get_messages(prompt))
            print("\nDone")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to perform all tasks, error: {e}")
            continue

    ros_client.terminate()


def local_example(system_init: Tuple[roslibpy.Ros, argparse.Namespace], prompt_init: Dict[str, str], model_init: Dict[str, str]) -> None:
    from utils import Recorder
    from ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, Bark, TTSParams
    from pynput.keyboard import Key
    from commander import Agent

    import socket

    ros_client: roslibpy.Ros = system_init[0]
    args: argparse.Namespace = system_init[1]
    input_recording: str = "input_recording.wav"

    ros_agent = Agent(
        WhisperCPP(STTParams(
            model_path=model_init["stt"],
            initial_prompt="",
            server_hostname=socket.gethostbyname(socket.gethostname()),
            server_port=8080
        )),
        LlamaCPP(LLMParams(
            model_path=model_init["llm"],
            initial_prompt=prompt_init["ros"],
            n_of_tokens_to_predict=-1,  # gauge this reasonably
            n_of_gpu_layers_to_offload=43,
            grammar_file_path=str(os.path.realpath(__file__).rstrip(os.path.basename(__file__))) + 'grammars/posestamped.gbnf',
            server_hostname=socket.gethostbyname(socket.gethostname()),
            server_port=8081,
            n_of_parallel_server_requests=1
        ))
    ).launch()

    chat_agent = Agent(
        WhisperCPP(STTParams(
            model_path=model_init["stt"],
            initial_prompt="",
            server_hostname=socket.gethostbyname(socket.gethostname()),
            server_port=8082
        )),
        LlamaCPP(LLMParams(
            model_path=model_init["llm"],
            initial_prompt=prompt_init["chat"],
            n_of_tokens_to_predict=-1,  # gauge this reasonably
            n_of_gpu_layers_to_offload=43,
            server_hostname=socket.gethostbyname(socket.gethostname()),
            server_port=8083,
            n_of_parallel_server_requests=1
        )),
        Bark(TTSParams(
            model_path=model_init["tts"],
            voice=model_init["tts_voice"]
        ))).launch()

    while True:
        try:
            print("\nPress and hold 'space' to record your command ...\n")
            # FIXME(recording): make it inline instead of this crappy file saving
            with Recorder(frequency_hz=16000) as r:
                r.hold_to_record(Key.space)
                r.save_recording(input_recording)
            print(f"\nDone, created file: {input_recording}")

            print("\nResponding ...")
            # chat_agent.tts.synthesize(chat_agent.llm.respond(chat_agent.stt.transcribe(input_recording, load_model=True), inline_response=True, load_model=True))
            # print(f"\nPlaying back the response ...")
            # chat_agent.tts.play_synthesis()
            chat_agent.respond(input_recording)
            print("\nDone")

            print(f"\nsummary:\n\t{input_recording} -> "
                  f"(whisper)\n\t-> '{chat_agent.stt.last_transcription}' -> "
                  f"(llama)\n\t-> '{chat_agent.llm.last_response}' -> "
                  f"(bark)\n\t-> {chat_agent.tts.generated_file}")

            print("\nGenerating commands ...")
            # messages = json.loads(ros_agent.llm.respond(ros_agent.stt.transcribe(input_recording, load_model=True), load_model=True))["commands"]
            messages = json.loads(ros_agent.respond(input_recording))["commands"]
            print("\nDone")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f"Publishing message: '{message}'")
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to perform all tasks, error: {e}")
            continue

    ros_client.terminate()


if __name__ == '__main__':
    # models: str = "/run/user/1000/gvfs/smb-share:server=starlink-nas.local,share=data%20nas/models"
    models: str = "/media/user/data_ssd/models"
    llm: str = models + "/llama3/Meta-Llama-3-8B/ggml-model-q4_0.gguf"
    stt: str = models + "/whisper/large/ggml-model-q4_0-large-v3.bin"
    tts: str = models + "/bark/"
    tts_voice: str = "announcer"

    openai_example(ros_factory(), prompt_factory())
    # local_example(ros_factory(), prompt_factory(), dict(stt=stt, llm=llm, tts=tts, tts_voice=tts_voice))
