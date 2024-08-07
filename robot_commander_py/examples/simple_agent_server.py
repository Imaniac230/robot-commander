from robot_commander_library.commander import Agent
from robot_commander_library.ai_interface import WhisperCPP, STTParams, LlamaCPP, LLMParams, BarkCPP, TTSParams
import netifaces as ni
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', type=str, default='lo', help='Network interface for servers.')
    args: argparse.Namespace = parser.parse_args()

    local_address: str = ni.ifaddresses(args.interface)[ni.AF_INET][0]['addr']
    models: str = "/media/user/data_ssd/models"
    llm: str = models + "/llama3/hf/Meta-Llama-3-8B-Instruct/ggml-model-q4_0.gguf"
    stt: str = models + "/whisper/large/ggml-model-q4_0-large-v3.bin"
    tts: str = models + "/bark/hf/bark/ggml-model-q4_0.bin"

    Agent(
        WhisperCPP(STTParams(
            model_path=stt,
            initial_prompt="",
            server_hostname=local_address,
            server_port=8080
        )),
        LlamaCPP(LLMParams(
            model_path=llm,
            initial_prompt="You are a helpful assistant that always gives precise and relevant answers.",
            n_of_tokens_to_predict=500,
            n_of_gpu_layers_to_offload=43,
            server_hostname=local_address,
            server_port=8081,
            n_of_parallel_server_requests=1
        )),
        BarkCPP(TTSParams(
            model_path=tts,
            voice="",# not used by local server yet
            server_hostname=local_address,
            server_port=8082
        ))
    ).launch()
