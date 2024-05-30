from typing import Tuple, Dict

import argparse
import time
import roslibpy
import glob
import os
import json


def local_model_factory() -> Dict[str, str]:
    # models: str = "/run/user/1000/gvfs/smb-share:server=starlink-nas.local,share=data%20nas/models"
    models: str = "/media/user/data_ssd/models"
    llm: str = models + "/llama3/Meta-Llama-3-8B/ggml-model-q4_0-fromhf-origtokenizer-with-convert-hf.gguf"
    stt: str = models + "/whisper/large/ggml-model-q4_0-large-v3.bin"
    tts: str = models + "/bark/"
    tts_voice: str = "announcer"
    return dict(stt=stt, llm=llm, tts=tts, tts_voice=tts_voice)


def init_factory() -> Tuple[argparse.Namespace, roslibpy.Ros, Dict[str, str]]:
    from utils import ROSPublisher, RobotChat

    parser = argparse.ArgumentParser()
    parser.add_argument('--net_interface', type=str, default='lo', help='Network interface for servers.')
    parser.add_argument('--api_key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--ros_host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--ros_port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--ros_topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--ros_message_type', type=str, required=True, help='Message type for the topic.')
    parser.add_argument('--robot_name', type=str, required=True, help='Name of the robot that will be used for the system prompt.')
    args: argparse.Namespace = parser.parse_args()

    ros = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
    ros.run()

    return args, ros, dict(chat=RobotChat('robot-chat.txt', name=args.robot_name).prompt(), ros=ROSPublisher('ros-publisher.txt').prompt())


def openai_example(system_init: Tuple[argparse.Namespace, roslibpy.Ros, Dict[str, str]]) -> None:
    from ai_interface import OpenAI, OpenAIParams

    args: argparse.Namespace = system_init[0]
    ros_client: roslibpy.Ros = system_init[1]
    prompt_init: Dict[str, str] = system_init[2]
    ros_agent = OpenAI(OpenAIParams(api_key=args.api_key, chat_initial_prompt=prompt_init["ros"]))
    chat_agent = OpenAI(OpenAIParams(api_key=args.api_key, chat_initial_prompt=prompt_init["chat"]))
    while True:
        try:
            print("\nPress and hold 'F10' to record your command ...\n")
            prompt: str = chat_agent.get_voice_prompt()

            print("\nResponding ...")
            response: str = chat_agent.get_messages(prompt)
            chat_agent.generate_audio(response)
            print("\nDone")

            print(f"\nsummary:\n\t"
                  f"(whisper)\n\t-> '{prompt}' -> "
                  f"(chatgpt)\n\t-> '{response}'")

            print("\nGenerating commands ...")
            messages = json.loads(ros_agent.get_messages(prompt))
            print("\nDone")

            publisher = roslibpy.Topic(ros_client, args.ros_topic, args.ros_message_type)
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


def local_example(system_init: Tuple[argparse.Namespace, roslibpy.Ros, Dict[str, str]], model_init: Dict[str, str]) -> None:
    from utils import Recorder
    from ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, Bark, TTSParams
    from pynput.keyboard import Key
    from commander import Agent

    import netifaces as ni

    args: argparse.Namespace = system_init[0]
    ros_client: roslibpy.Ros = system_init[1]
    prompt_init: Dict[str, str] = system_init[2]
    input_recording: str = "input_recording.wav"

    ros_agent = Agent(
        WhisperCPP(STTParams(
            model_path=model_init["stt"],
            initial_prompt="",
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8080
        )),
        LlamaCPP(LLMParams(
            model_path=model_init["llm"],
            initial_prompt=prompt_init["ros"],
            n_of_tokens_to_predict=-1,  # gauge this reasonably
            n_of_gpu_layers_to_offload=43,
            grammar_file_path=str(os.path.realpath(__file__).rstrip(os.path.basename(__file__))) + 'grammars/posestamped.gbnf',
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8081,
            n_of_parallel_server_requests=1
        ))
    ).launch()

    chat_agent = Agent(
        WhisperCPP(STTParams(
            model_path=model_init["stt"],
            initial_prompt="",
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8082
        )),
        LlamaCPP(LLMParams(
            model_path=model_init["llm"],
            initial_prompt=prompt_init["chat"],
            n_of_tokens_to_predict=-1,  # gauge this reasonably
            n_of_gpu_layers_to_offload=43,
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8083,
            n_of_parallel_server_requests=1
        )),
        Bark(TTSParams(
            model_path=model_init["tts"],
            voice=model_init["tts_voice"]
        ))).launch()

    while True:
        try:
            print("\nPress and hold 'F10' to record your command ...\n")
            # FIXME(recording): make it inline instead of this crappy file saving
            with Recorder(frequency_hz=16000) as r:
                r.hold_to_record(Key.f10)
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
    openai_example(init_factory())
    # local_example(init_factory(), local_model_factory())
