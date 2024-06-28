from ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, BarkCPP, TTSParams
from commander import Agent
from utils import ROSPublisher, RobotChat

import netifaces as ni
from typing import Optional
from pathlib import Path

import rclpy
from rclpy.node import Node


class AgentServer(Node):

    def __init__(self):
        super().__init__('agent_server')

        self.declare_parameter('type', 'CHAT')
        self.declare_parameter('server_hostname', '')

        self.declare_parameter('speech_to_text.port', 8080)
        self.declare_parameter('speech_to_text.model_file', '')

        self.declare_parameter('language_model.port', 8081)
        self.declare_parameter('language_model.model_file', '')
        self.declare_parameter('language_model.initial_prompt_file', '')
        self.declare_parameter('language_model.initial_prompt_context', '')

        self.declare_parameter('text_to_speech.port', 8082)
        self.declare_parameter('text_to_speech.model_file', '')
        self.declare_parameter('text_to_speech.voice_type', 'announcer')  # FIXME(tts-server): this is currently unused

        self.agent: Optional[Agent] = None

        self.initialize()

    def initialize(self):
        stt: str = self.get_parameter('speech_to_text.model_file').get_parameter_value().string_value
        if not Path(stt).is_file(): raise FileNotFoundError(f"Speech-to-text model file '{stt}' not found.")
        llm: str = self.get_parameter('language_model.model_file').get_parameter_value().string_value
        if not Path(llm).is_file(): raise FileNotFoundError(f"Language-model model file '{llm}' not found.")
        tts: str = self.get_parameter('text_to_speech.model_file').get_parameter_value().string_value
        if tts and not Path(tts).is_file(): raise FileNotFoundError(f"Text-to-speech model file '{tts}' not found.")

        agent_type: str = self.get_parameter('type').get_parameter_value().string_value
        llm_init_file: str = self.get_parameter('language_model.initial_prompt_file').get_parameter_value().string_value
        llm_context: str = self.get_parameter('language_model.initial_prompt_context').get_parameter_value().string_value
        llm_init: str = ""
        llm_max_tokens: int = -1
        if Path(llm_init_file).is_file():
            if agent_type == "CHAT":
                llm_init = RobotChat(llm_init_file, personality=llm_context if llm_context else None).prompt()
                llm_max_tokens = 50
            elif agent_type == "ROS":
                llm_init = ROSPublisher(llm_init_file, environment=llm_context if llm_context else None).prompt()
                llm_max_tokens = 500
            else:
                self.get_logger().warning(f"Unsupported agent type: '{agent_type}'. Using an empty initial prompt.")
        else:
            self.get_logger().warning(f"Language-model initial prompt file '{llm_init_file}' not found, initial prompt will not be used.")

        hostname: str = self.get_parameter('server_hostname').get_parameter_value().string_value
        if not hostname:
            hostname = ni.ifaddresses('lo')[ni.AF_INET][0]['addr']
            self.get_logger().warning(f"Server hostname was not provided, using local interface address: {hostname}.")

        self.agent = Agent(
            WhisperCPP(STTParams(
                model_path=stt,
                initial_prompt="",
                server_hostname=hostname,
                server_port=self.get_parameter('speech_to_text.port').get_parameter_value().integer_value
            )),
            LlamaCPP(LLMParams(
                model_path=llm,
                initial_prompt=llm_init,
                n_of_tokens_to_predict=llm_max_tokens,
                n_of_gpu_layers_to_offload=20,  # FIXME(gpu): this should be configured externally
                server_hostname=hostname,
                server_port=self.get_parameter('language_model.port').get_parameter_value().integer_value,
                n_of_parallel_server_requests=1
            )),
            # TODO(tts-server): local tts server is only experimental for now
            BarkCPP(TTSParams(
                model_path=tts,
                voice=self.get_parameter('text_to_speech.voice_type').get_parameter_value().string_value,
                server_hostname=hostname,
                server_port=self.get_parameter('text_to_speech.port').get_parameter_value().integer_value
            )) if tts else None
        )

        self.launch()

    def launch(self):
        if self.agent is not None:
            self.agent.launch()
        else:
            raise RuntimeError("Agent was not initialized.")


def main(args=None):
    rclpy.init(args=args)

    server = AgentServer()
    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
