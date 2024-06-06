from utils import Recorder
from commander import Commander, CommanderParams
from ai_interface import Bark, TTSParams
from pynput.keyboard import Key

import argparse
import json
import roslibpy
import time


#TODO(ros): this example should be implemented as the main ROS node for this project and it shouldn't have to be limited only to python in any way,
#   also expose all relevant parameters into the config
#FIXME(oai-requests): raw requests to the oai API will not work properly here yet, the system prompt will have to be provided from here
#   together with the user request, however, the local models are already initialized with the system prompt during server launch
def handle_requests() -> None:
    #TODO(tts-server): local tts server is not yet supported, must load the model with each prompt for now
    bark = Bark(TTSParams(model_path=args.tts_model_path, voice="announcer"))

    input_recording: str = "input_recording.wav"
    if args.use_local:
        chat_commander = Commander(
            CommanderParams(
                stt_host="http://" + args.local_address + ":8082",
                stt_endpoint="/inference",
                llm_host="http://" + args.local_address + ":8083",
                llm_endpoint="/v1/chat/completions",
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
                tts_voice="fable",
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
            #TODO(tts-server): local tts server is not yet supported, must load the model with each prompt for now
            response = chat_commander.respond(input_recording, playback_response=not args.use_local, response_format=None)
            if args.use_local:
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
            messages = json.loads(ros_commander.respond(input_recording, response_format="grammars/posestamped.json"))
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
    parser.add_argument('--use_local', type=bool, default=False, help='Use local servers for requests. If false, will use OpenAI servers.')
    parser.add_argument('--local_address', type=str, default='127.0.0.1', help='Address for local server requests.')
    parser.add_argument('--api_key', type=str, default=None, help='OpenAI or custom (if used for local models) API key.')
    parser.add_argument('--ros_host', type=str, default='localhost', help='ROS bridge host.')
    parser.add_argument('--ros_port', type=int, default='9090', help='ROS bridge port.')
    parser.add_argument('--ros_topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--ros_message_type', type=str, required=True, help='Message type for the topic.')
    #TODO(tts-server): local tts server is not yet supported, must load the model with each prompt for now
    parser.add_argument('--tts_model_path', type=str, required=True, help='Path to the local tts model files (this must currently be provided, as the server is not utilized).')
    args: argparse.Namespace = parser.parse_args()

    ros_client = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
    ros_client.run()

    handle_requests()

    ros_client.terminate()
