from typing import Tuple, Dict

import argparse
import time
import roslibpy
import os
import json


def local_model_factory() -> Dict[str, str]:
    # models: str = "/run/user/1000/gvfs/smb-share:server=starlink-nas.local,share=data%20nas/models"
    models: str = "/media/user/data_ssd/models"
    llm: str = models + "/llama3/hf/Meta-Llama-3-8B-Instruct/ggml-model-q4_0.gguf"
    stt: str = models + "/whisper/large/ggml-model-q4_0-large-v3.bin"
    #TODO(tts-server): test the experimental bark.cpp server
    # tts: str = models + "/bark/hf/bark/"
    tts: str = models + "/bark/hf/bark/ggml-model-q4_0.bin"
    tts_voice: str = "announcer"
    return dict(stt=stt, llm=llm, tts=tts, tts_voice=tts_voice)


def init_factory() -> Tuple[argparse.Namespace, roslibpy.Ros, Dict[str, str]]:
    from utils import ROSPublisher, RobotChat

    parser = argparse.ArgumentParser()
    parser.add_argument('--load_models', action='store_true', default=False, help='Load models for each prompt instead of spawning servers.')
    parser.add_argument('--net_interface', type=str, default='lo', help='Network interface for servers.')
    parser.add_argument('--api_key', type=str, default=None, help='OpenAI or custom (if used for local models) API key.')
    parser.add_argument('--ros_host', type=str, default='localhost', help='ROS bridge host.')
    parser.add_argument('--ros_port', type=int, default='9090', help='ROS bridge port.')
    parser.add_argument('--ros_topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--ros_message_type', type=str, required=True, help='Message type for the topic.')
    args: argparse.Namespace = parser.parse_args()

    ros = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
    ros.run()

    return args, ros, dict(chat=RobotChat('robot-chat.txt').prompt(), ros=ROSPublisher('ros-publisher.txt').prompt())


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
            prompt: str = chat_agent.transcribe()
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to record the prompt, error: {e}")
            continue

        try:
            print("\nResponding ...")
            response: str = chat_agent.respond(prompt)
            chat_agent.synthesize(response)
            print("\nDone")

            print(f"\nsummary:\n\t"
                  f"(whisper)\n\t-> '{prompt}' -> "
                  f"(chatgpt)\n\t-> '{response}'")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete chat agent tasks: {e}")
            print("Continuing to the ROS agent")
            pass

        try:
            print("\nGenerating commands ...")
            messages = json.loads(ros_agent.respond(prompt))
            print("\nDone")

            publisher = roslibpy.Topic(ros_client, args.ros_topic, args.ros_message_type)
            if ros_client.is_connected:
                for message in messages:
                    stamp = roslibpy.Time.now()
                    message["header"]["stamp"]["sec"] = stamp.secs
                    message["header"]["stamp"]["nanosec"] = stamp.nsecs
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete ROS agent tasks, error: {e}")
            continue

    ros_client.terminate()


def local_example(system_init: Tuple[argparse.Namespace, roslibpy.Ros, Dict[str, str]], model_init: Dict[str, str]) -> None:
    from utils import Recorder
    from ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, Bark, BarkCPP, TTSParams
    from pynput.keyboard import Key
    from commander import Agent
    import netifaces as ni

    args: argparse.Namespace = system_init[0]
    ros_client: roslibpy.Ros = system_init[1]
    prompt_init: Dict[str, str] = system_init[2]
    input_recording: str = "input_recording.wav"

    #TODO(efficient-agents): We wouldn't need two separate stt instances here, as they are identical,
    #   but the current design doesn't support any cross-sharing between independent agents.
    #   If the requests are made from an external requestor (which should be the target case), then one of them could be omitted.
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
            n_of_tokens_to_predict=500,
            n_of_gpu_layers_to_offload=20,
            json_schema_file_path=str(os.path.realpath(__file__).rstrip(os.path.basename(__file__))) + 'grammars/posestamped.json',
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8081,
            n_of_parallel_server_requests=1
        ))
    )
    if not args.load_models: ros_agent.launch()

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
            n_of_tokens_to_predict=100,
            n_of_gpu_layers_to_offload=20,
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8083,
            n_of_parallel_server_requests=1
        )),
        #TODO(tts-server): import server for tts once it is reasonably available
        # Bark(TTSParams(
        #     model_path=model_init["tts"],
        #     voice=model_init["tts_voice"]
        # )),
        BarkCPP(TTSParams(
            model_path=model_init["tts"],
            voice=model_init["tts_voice"],# not used by local server yet
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8084
        ))
    )
    if not args.load_models: chat_agent.launch()

    while True:
        try:
            print("\nPress and hold 'F10' to record your command ...\n")
            # FIXME(recording): make it inline instead of this crappy file saving
            with Recorder(frequency_hz=16000) as r:
                r.hold_to_record(Key.f10)
                r.save_recording(input_recording)
            print(f"\nDone, created file: {input_recording}")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to record the prompt, error: {e}")
            continue

        try:
            print("\nResponding ...")
            chat_agent.respond(input_recording, load_models=args.load_models, playback_response=True)
            print("\nDone")

            print(f"\nsummary:\n\t{input_recording} -> "
                  f"(whisper)\n\t-> '{chat_agent.stt.last_transcription}' -> "
                  f"(llama)\n\t-> '{chat_agent.llm.last_response}' -> "
                  f"(bark)\n\t-> {chat_agent.tts.generated_file}")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete chat agent tasks: {e}")
            print("Continuing to the ROS agent")
            pass

        try:
            print("\nGenerating commands ...")
            messages = json.loads(ros_agent.respond(input_recording, load_models=args.load_models))
            print("\nDone")

            publisher = roslibpy.Topic(ros_client, args.ros_topic, args.ros_message_type)
            if ros_client.is_connected:
                for message in messages:
                    stamp = roslibpy.Time.now()
                    message["header"]["stamp"]["sec"] = stamp.secs
                    message["header"]["stamp"]["nanosec"] = stamp.nsecs
                    publisher.publish(roslibpy.Message(message))
                    print(f"Publishing message: '{message}'")
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete ROS agent tasks, error: {e}")
            continue

    ros_client.terminate()


if __name__ == '__main__':
    # openai_example(init_factory())
    local_example(init_factory(), local_model_factory())
