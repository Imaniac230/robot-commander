from robot_commander_library.commander import Commander, CommanderParams
from robot_commander_library.ai_interface import Bark, TTSParams
from robot_commander_library.utils import ROSPublisher, RobotChat

from pathlib import Path
from typing import Optional
from enum import Enum, unique

from rclpy.node import Node


@unique
class AgentType(Enum):
    CHAT = 1
    ROS = 2


class CommanderActionServerInterface(Node):
    def __init__(self, name: str, agent_type: AgentType):
        super().__init__(name)

        self.declare_parameter('speech_to_text.host', '')
        self.declare_parameter('speech_to_text.api_key', '')
        self.declare_parameter('speech_to_text.name', '')

        self.declare_parameter('language_model.host', '')
        self.declare_parameter('language_model.api_key', '')
        self.declare_parameter('language_model.name', '')
        if agent_type == AgentType.ROS:
            self.declare_parameter('language_model.grammar_file', '')
            self.declare_parameter('language_model.ros_messages_path', '')
        self.declare_parameter('language_model.initial_prompt_file', '')
        self.declare_parameter('language_model.initial_prompt_context', '')

        if agent_type == AgentType.CHAT:
            self.declare_parameter('text_to_speech.host', '')
            self.declare_parameter('text_to_speech.api_key', '')
            self.declare_parameter('text_to_speech.name', '')
            self.declare_parameter('text_to_speech.voice_type', '')
            self.declare_parameter('text_to_speech.pytorch_model_path', '')
            self.declare_parameter('text_to_speech.use_pytorch', False)

        self.type: AgentType = agent_type
        self.commander: Optional[Commander] = None

        self.system_prompt: Optional[str] = None
        self.grammar_file: Optional[str] = None
        self.pytorch_tts: Optional[Bark] = None

    def initialize(self):
        stt_host: str = self.get_parameter('speech_to_text.host').get_parameter_value().string_value
        if not stt_host: raise ValueError("Speech-to-text host not provided.")
        llm_host: str = self.get_parameter('language_model.host').get_parameter_value().string_value
        if not llm_host: raise ValueError("Language-model host not provided.")

        tts_host: str = ""
        if self.type == AgentType.CHAT: tts_host = self.get_parameter('text_to_speech.host').get_parameter_value().string_value

        stt_api_key: str = self.get_parameter('speech_to_text.api_key').get_parameter_value().string_value
        if any(service in stt_host.lower() for service in ["openai", "anthropic"]):
            if not stt_api_key: raise ValueError(f"External speech-to-text host '{stt_host}' requires an API key, but no key was specified.")
        llm_api_key: str = self.get_parameter('language_model.api_key').get_parameter_value().string_value
        if any(service in llm_host.lower() for service in ["openai", "anthropic"]):
            if not llm_api_key: raise ValueError(f"External language-model host '{llm_host}' requires an API key, but no key was specified.")
        tts_api_key: str = ""
        if self.type == AgentType.CHAT:
            tts_api_key = self.get_parameter('text_to_speech.api_key').get_parameter_value().string_value
            if any(service in tts_host.lower() for service in ["openai", "anthropic"]):
                if not tts_api_key: raise ValueError(f"External text-to-speech host '{tts_host}' requires an API key, but no key was specified.")

        # TODO(http-endpoints): We should create explicit wrappers for all supported providers (openai, anthropic, local *.cpp, and any other)
        #   that would always create and output the required endpoint and payload data formats implicitly. The wrappers could then be
        #   initialized with the hostname here instead of this.
        if "openai" in stt_host.lower():
            stt_endpoint: str = "/v1/audio/translations"
        else:
            stt_endpoint: str = "/inference"

        if "anthropic" in llm_host.lower():
            llm_endpoint: str = "/v1/messages"
        else:  # local server uses the same endpoint as openai
            llm_endpoint: str = "/v1/chat/completions"

        tts_endpoint: str = ""
        if self.type == AgentType.CHAT:
            # local server uses the same endpoint as openai
            # TODO(tts-server): local tts server is only experimental for now
            tts_endpoint = "/v1/audio/speech"

        if self.type == AgentType.ROS:
            grammar_file: str = self.get_parameter('language_model.grammar_file').get_parameter_value().string_value
            if grammar_file and "openai" in llm_host.lower():
                self.get_logger().info("Grammar/schema constraints are currently not required when accessing OpenAI models and will be ignored.")
            elif grammar_file:
                if Path(grammar_file).is_file():
                    self.grammar_file = grammar_file
                else:
                    self.get_logger().warning(f"Grammar file '{grammar_file}' not found, will not use grammar constraints.")

        if any(api in llm_host.lower() for api in ["openai", "anthropic"]):
            llm_init_file: str = self.get_parameter('language_model.initial_prompt_file').get_parameter_value().string_value
            llm_context: str = self.get_parameter('language_model.initial_prompt_context').get_parameter_value().string_value
            if Path(llm_init_file).is_file():
                if self.type == AgentType.CHAT:
                    self.system_prompt = RobotChat(llm_init_file, personality=llm_context if llm_context else None).prompt()
                elif self.type == AgentType.ROS:
                    ros_messages_dir: str = self.get_parameter('language_model.ros_messages_path').get_parameter_value().string_value
                    if not ros_messages_dir or not Path(ros_messages_dir).is_dir(): self.get_logger().warn(f"Directory '{ros_messages_dir}' not found, ROS message definitions will not be specified.")
                    self.system_prompt = ROSPublisher(llm_init_file, ros_messages_dir, environment=llm_context if llm_context else None).prompt()
                else:
                    self.get_logger.warn(f"Agent type '{self.type}' is not supported, a system prompt will not be created.")
            else:
                self.get_logger().warning(f"External API language-model requires an initial prompt but file '{llm_init_file}' was not found.")

        stt_name: str = self.get_parameter('speech_to_text.name').get_parameter_value().string_value
        llm_name: str = self.get_parameter('language_model.name').get_parameter_value().string_value

        tts_name: str = ""
        tts_voice: str = ""
        if self.type == AgentType.CHAT:
            tts_name = self.get_parameter('text_to_speech.name').get_parameter_value().string_value
            tts_voice = self.get_parameter('text_to_speech.voice_type').get_parameter_value().string_value

            if self.get_parameter('text_to_speech.use_pytorch').get_parameter_value().bool_value:
                model_path: str = self.get_parameter('text_to_speech.pytorch_model_path').get_parameter_value().string_value
                if not model_path or not Path(model_path).is_dir(): raise ValueError(f"Text-to-speech pytorch model loading is enabled, but the provided model path '{model_path}' was not found.")
                self.pytorch_tts = Bark(TTSParams(model_path=model_path, voice=tts_voice if tts_voice is not None else "announcer"))

        self.commander = Commander(
            CommanderParams(
                stt_host=stt_host,
                stt_api_key=stt_api_key if stt_api_key else None,
                stt_endpoint=stt_endpoint,
                stt_name=stt_name if stt_name else None,
                llm_host=llm_host,
                llm_api_key=llm_api_key if llm_api_key else None,
                llm_endpoint=llm_endpoint,
                llm_name=llm_name if llm_name else None,
                tts_host=tts_host if tts_host and self.pytorch_tts is None else None,
                tts_api_key=tts_api_key if tts_api_key else None,
                tts_endpoint=tts_endpoint if tts_host else None,
                tts_voice=tts_voice if tts_voice else None,
                tts_name=tts_name if tts_name else None
            ))
