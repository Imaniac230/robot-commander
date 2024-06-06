from ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, Bark, TTSParams
from commander import Agent
from utils import ROSPublisher, RobotChat

import netifaces as ni
import argparse
import os


#TODO(ros): we can wrap this into a Python ros node and also expose all relevant parameters into the config,
#   net-interface, ports, model-paths, grammar-files, initial-prompt-files, ...
def launch_agents() -> None:
    #TODO(redundant-agents): We wouldn't need two separate stt instances here, as they are identical,
    #   but the current design doesn't support any cross-sharing between independent agents.
    #   If the requests are made from an external requestor (which should be the target case), then one of them could be omitted.
    ros_agent = Agent(
        WhisperCPP(STTParams(
            model_path=args.stt_model_file,
            initial_prompt="",
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8080
        )),
        LlamaCPP(LLMParams(
            model_path=args.llm_model_file,
            initial_prompt=ROSPublisher('ros-publisher.txt').prompt(),
            n_of_tokens_to_predict=500,
            n_of_gpu_layers_to_offload=20,
            json_schema_file_path=str(os.path.realpath(__file__).rstrip(os.path.basename(__file__))) + 'grammars/posestamped.json',
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8081,
            n_of_parallel_server_requests=1
        ))
    ).launch()

    chat_agent = Agent(
        WhisperCPP(STTParams(
            model_path=args.stt_model_file,
            initial_prompt="",
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8082
        )),
        LlamaCPP(LLMParams(
            model_path=args.llm_model_file,
            initial_prompt=RobotChat('robot-chat.txt').prompt(),
            n_of_tokens_to_predict=100,
            n_of_gpu_layers_to_offload=20,
            server_hostname=ni.ifaddresses(args.net_interface)[ni.AF_INET][0]['addr'],
            server_port=8083,
            n_of_parallel_server_requests=1
        )),
        #TODO(tts-server): import server for tts once it is reasonably available
        Bark(TTSParams(
            model_path=args.tts_model_path,
            voice="announcer"
        ))).launch()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--net_interface', type=str, default='lo', help='Network interface for servers.')
    parser.add_argument('--stt_model_file', type=str, required=True, help='Path to the local stt model file.')
    parser.add_argument('--llm_model_file', type=str, required=True, help='Path to the local llm model file.')
    parser.add_argument('--tts_model_path', type=str, required=True, help='Path to the local tts model files (this is currently not used as a server).')
    args: argparse.Namespace = parser.parse_args()

    launch_agents()
