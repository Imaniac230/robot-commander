from commander import Commander, CommanderParams
from ai_interface import Bark, TTSParams
from utils import ROSPublisher, RobotChat

from pathlib import Path
from typing import Optional

from rclpy.node import Node


class CommanderActionServerInterface(Node):

    def __init__(self, name: str, is_chat: bool):
        super().__init__(name)

        self.declare_parameter('api_key', '')  # FIXME(multi-api-keys): use the individual component keys instead

        self.declare_parameter('speech_to_text.host', '')
        self.declare_parameter('speech_to_text.api_key', '')
        self.declare_parameter('speech_to_text.name', '')

        self.declare_parameter('language_model.host', '')
        self.declare_parameter('language_model.api_key', '')
        self.declare_parameter('language_model.name', '')
        if not is_chat: self.declare_parameter('language_model.grammar_file', '')
        self.declare_parameter('language_model.initial_prompt_file', '')
        self.declare_parameter('language_model.initial_prompt_context', '')

        if is_chat:
            self.declare_parameter('text_to_speech.host', '')
            self.declare_parameter('text_to_speech.api_key', '')
            self.declare_parameter('text_to_speech.name', '')
            self.declare_parameter('text_to_speech.voice_type', '')
            self.declare_parameter('text_to_speech.pytorch_model_path', '')
            self.declare_parameter('text_to_speech.use_pytorch', False)

        self.is_chat: bool = is_chat
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
        if self.is_chat: tts_host = self.get_parameter('text_to_speech.host').get_parameter_value().string_value

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
        if self.is_chat:
            # local server uses the same endpoint as openai
            # TODO(tts-server): local tts server is only experimental for now
            tts_endpoint = "/v1/audio/speech"

        if not self.is_chat:
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
                self.system_prompt = RobotChat(
                    llm_init_file,
                    personality=llm_context if llm_context else None
                ).prompt() if self.is_chat else ROSPublisher(
                    llm_init_file,
                    environment=llm_context if llm_context else None
                ).prompt()
            else:
                self.get_logger().warning(f"External API language-model requires an initial prompt but file '{llm_init_file}' was not found.")

        api_key: str = self.get_parameter('api_key').get_parameter_value().string_value
        stt_name: str = self.get_parameter('speech_to_text.name').get_parameter_value().string_value
        llm_name: str = self.get_parameter('language_model.name').get_parameter_value().string_value

        tts_name: str = ""
        tts_voice: str = ""
        if self.is_chat:
            tts_name = self.get_parameter('text_to_speech.name').get_parameter_value().string_value
            tts_voice = self.get_parameter('text_to_speech.voice_type').get_parameter_value().string_value

            if self.get_parameter('text_to_speech.use_pytorch').get_parameter_value().bool_value:
                model_path: str = self.get_parameter('text_to_speech.pytorch_model_path').get_parameter_value().string_value
                if not model_path or not Path(model_path).is_dir(): raise ValueError(f"Text-to-speech pytorch model loading is enabled, but the provided model path '{model_path}' was not found.")
                self.pytorch_tts = Bark(TTSParams(model_path=model_path, voice=tts_voice if tts_voice is not None else "announcer"))

        self.commander = Commander(
            CommanderParams(
                stt_host=stt_host,
                stt_endpoint=stt_endpoint,
                stt_name=stt_name if stt_name else None,
                llm_host=llm_host,
                llm_endpoint=llm_endpoint,
                llm_name=llm_name if llm_name else None,
                tts_host=tts_host if tts_host and self.pytorch_tts is None else None,
                tts_endpoint=tts_endpoint if tts_host else None,
                tts_voice=tts_voice if tts_voice else None,
                tts_name=tts_name if tts_name else None,
                # FIXME(multi-api-keys): refactor commander to take separate keys for each part?
                api_key=api_key if api_key else None
            ))
