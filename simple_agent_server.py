from commander import Agent
from ai_interface import WhisperCPP, STTParams, LlamaCPP, LLMParams
import netifaces as ni
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', type=str, default='lo', help='Network interface for servers.')
    args: argparse.Namespace = parser.parse_args()

    local_address: str = ni.ifaddresses(args.interface)[ni.AF_INET][0]['addr']
    models: str = "/media/user/data_ssd/models"
    llm: str = models + "/llama2/original/llama-2-13b-chat/ggml-model-q4_0.gguf"
    stt: str = models + "/whisper/large/ggml-model-q4_0-large-v3.bin"

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
        ))
    ).launch()
