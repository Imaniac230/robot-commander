from robot_commander_library.ai_interface import LlamaCPP, LLMParams, WhisperCPP, STTParams, BarkCPP, TTSParams
from robot_commander_library.commander import Agent
from robot_commander_library.utils import ROSPublisher, RobotChat

from typing import Optional
from pathlib import Path
import netifaces as ni
import time

import rclpy
from rclpy.node import Node


class AgentServer(Node):

    def __init__(self):
        super().__init__('agent_server')

        self.declare_parameter('type', 'CHAT')
        is_ros: bool = self.get_parameter('type').get_parameter_value().string_value == 'ROS'
        self.declare_parameter('server_hostname', '')

        self.declare_parameter('speech_to_text.port', 8080)
        self.declare_parameter('speech_to_text.model_file', '')

        self.declare_parameter('language_model.port', 8081)
        self.declare_parameter('language_model.model_file', '')
        if is_ros: self.declare_parameter('language_model.ros_messages_path', '')
        self.declare_parameter('language_model.initial_prompt_file', '')
        self.declare_parameter('language_model.initial_prompt_context', '')
        self.declare_parameter('language_model.max_output_tokens', -1)
        self.declare_parameter('language_model.gpu_offload_layers', 0)

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

        llm_max_tokens: int = self.get_parameter('language_model.max_output_tokens').get_parameter_value().integer_value
        # -2 -> until context filled, -1 -> infinite
        if llm_max_tokens < -2:
            self.get_logger().warning(f"Invalid language model 'max_output_tokens' value specified: {llm_max_tokens}, defaulting to -1 (unlimited).")
            llm_max_tokens = -1

        llm_init: str = ""
        if Path(llm_init_file).is_file():
            if agent_type == "CHAT":
                llm_init = RobotChat(llm_init_file, personality=llm_context if llm_context else None).prompt()
            elif agent_type == "ROS":
                ros_messages_dir: str = self.get_parameter('language_model.ros_messages_path').get_parameter_value().string_value
                if not ros_messages_dir or not Path(ros_messages_dir).is_dir(): self.get_logger().warn(f"Directory '{ros_messages_dir}' not found, ROS message definitions will not be specified.")
                llm_init = ROSPublisher(llm_init_file, ros_messages_dir, environment=llm_context if llm_context else None).prompt()
            else:
                self.get_logger().warning(f"Unsupported agent type: '{agent_type}'. Using an empty initial prompt.")
        else:
            self.get_logger().warning(f"Language-model initial prompt file '{llm_init_file}' not found, initial prompt will not be used.")

        hostname: str = self.get_parameter('server_hostname').get_parameter_value().string_value
        if not hostname:
            hostname = ni.ifaddresses('lo')[ni.AF_INET][0]['addr']
            self.get_logger().warning(f"Server hostname was not provided, using local interface address: {hostname}.")

        llm_gpu_layers: int = self.get_parameter('language_model.gpu_offload_layers').get_parameter_value().integer_value
        if llm_gpu_layers < 0:
            self.get_logger().warning(f"Invalid number of language model GPU layers for offloading: {llm_gpu_layers}, defaulting to 0.")
            llm_gpu_layers = 0

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
                n_of_gpu_layers_to_offload=llm_gpu_layers,
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

        while self.agent.alive(): time.sleep(0.1)
        raise RuntimeError("Agent failed to run successfully, terminating.")


def main(args=None):
    rclpy.init(args=args)

    server = AgentServer()
    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
