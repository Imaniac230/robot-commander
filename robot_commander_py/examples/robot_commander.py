from robot_commander_library.utils import Recorder, ROSPublisher, RobotChat
from robot_commander_library.commander import Commander, CommanderParams
from robot_commander_library.ai_interface import Bark, TTSParams
from pynput.keyboard import Key
from typing import Optional

import argparse
import json
import roslibpy
import time
import os


#TODO(ros): this example should be implemented as the main ROS node for this project and it shouldn't have to be limited only to python in any way,
#   also expose all relevant parameters into the config
def handle_requests() -> None:
    #TODO(tts-server): local tts server is only experimental for now, allow loading the pytorch model with each prompt as well
    bark: Optional[Bark] = None
    if args.pytorch_tts_model_path is not None: bark = Bark(TTSParams(model_path=args.pytorch_tts_model_path, voice=args.chat_voice if args.chat_voice is not None else "announcer"))

    input_recording: str = base_path + "input_recording.wav"
    if args.use_local:
        chat_commander = Commander(
            CommanderParams(
                stt_host="http://" + args.local_address + ":8082",
                stt_endpoint="/inference",
                llm_host="http://" + args.local_address + ":8083",
                llm_endpoint="/v1/chat/completions",
                tts_host="http://" + args.local_address + ":8084" if args.pytorch_tts_model_path is None else None,
                tts_endpoint="/v1/audio/speech" if args.pytorch_tts_model_path is None else None,
                tts_voice=args.chat_voice if args.chat_voice is not None else "announcer"
        ))
        ros_commander = Commander(
            CommanderParams(
                stt_host="http://" + args.local_address + ":8080",
                stt_endpoint="/inference",
                llm_host="http://" + args.local_address + ":8081",
                llm_endpoint="/v1/chat/completions",
        ))
    else:
        chat_commander = Commander(
            CommanderParams(
                stt_host="https://api.openai.com",
                stt_endpoint="/v1/audio/translations",
                stt_name="whisper-1",
                llm_host="https://api.openai.com",
                llm_endpoint="/v1/chat/completions",
                llm_name="gpt-4o",
                tts_host="https://api.openai.com",
                tts_endpoint="/v1/audio/speech",
                tts_voice=args.chat_voice if args.chat_voice is not None else "fable",
                tts_name="tts-1",
                api_key=args.api_key
        ))
        ros_commander = Commander(
            CommanderParams(
                stt_host="https://api.openai.com",
                stt_endpoint="/v1/audio/translations",
                stt_name="whisper-1",
                llm_host="https://api.openai.com",
                llm_endpoint="/v1/chat/completions",
                llm_name="gpt-4o",
                api_key=args.api_key
        ))

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
            #TODO(tts-server): local tts server is only experimental for now, allow loading the pytorch model with each prompt as well
            response = chat_commander.respond(
                input_recording,
                playback_response=bark is None,
                system_prompt=RobotChat(base_path + "prompts/robot-chat.txt", personality=args.personality_context).prompt() if not args.use_local else None
            )
            if bark is not None:
                bark.synthesize(response, load_model=True)
                bark.play_synthesis()
            print("\nDone")

            print(f"\nsummary:\n\t{input_recording} -> "
                  f"({chat_commander.params.stt_name if chat_commander.params.stt_name is not None else 'whisper'})\n\t-> '{chat_commander.last_transcription}' -> "
                  f"({chat_commander.params.llm_name if chat_commander.params.llm_name is not None else 'llama'})\n\t-> '{chat_commander.last_response}' -> "
                  f"({chat_commander.params.tts_name if chat_commander.params.tts_name is not None else 'bark'})\n\t-> {bark.generated_file}")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Failed to complete chat agent tasks: {e}")
            print("Continuing to the ROS agent")
            pass

        try:
            print("\nGenerating commands ...")
            messages = json.loads(ros_commander.respond(
                input_recording,
                response_format=base_path + "grammars/posestamped.json" if args.use_local else None, #NOTE: the restricted oai output doesn't seem to be wrapping the messages in an array correctly
                system_prompt=ROSPublisher(base_path + "prompts/ros-publisher.txt", base_path + "messages", environment=args.environment_context).prompt() if not args.use_local else None
            ))
            print("\nDone")

            print(f"\nsummary:\n\t{input_recording} -> "
                  f"({ros_commander.params.stt_name if ros_commander.params.stt_name is not None else 'whisper'})\n\t-> '{ros_commander.last_transcription}' -> "
                  f"({ros_commander.params.llm_name if ros_commander.params.llm_name is not None else 'llama'})\n\t-> '{ros_commander.last_response}'")

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_local', action='store_true', default=False, help='Use local servers for requests. If false, will use OpenAI servers.')
    parser.add_argument('--local_address', type=str, default='127.0.0.1', help='Address for local server requests.')
    parser.add_argument('--api_key', type=str, default=None, help='OpenAI or custom (if used for local models) API key.')
    parser.add_argument('--ros_host', type=str, default='localhost', help='ROS bridge host.')
    parser.add_argument('--ros_port', type=int, default='9090', help='ROS bridge port.')
    parser.add_argument('--ros_topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--ros_message_type', type=str, required=True, help='Message type for the topic.')
    parser.add_argument('--chat_voice', type=str, default=None, help='Voice type to be used by the tts. Note that local SunoAI and remote OpenAI use different options. For local, this is only valid when using the pytorch models here.')
    #TODO(tts-server): local tts server is only experimental for now, allow loading the pytorch model with each prompt as well
    parser.add_argument('--pytorch_tts_model_path', type=str, default=None, help='Path to the local pytorch tts model files. Use this for better results, the server is currently only experimental.')
    #this is only necessary if using the oai API
    parser.add_argument('--environment_context', type=str, default=None, help='Additional information about the agent is deployed in (required only for OpenAI requests).')
    parser.add_argument('--personality_context', type=str, default=None, help='Additional information about the agent personality (required only for OpenAI requests).')
    args: argparse.Namespace = parser.parse_args()

    ros_client = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
    ros_client.run()

    base_path: str = str(os.path.realpath(__name__).rstrip(os.path.basename(__name__)))

    handle_requests()

    ros_client.terminate()
