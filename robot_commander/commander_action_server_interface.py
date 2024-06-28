from commander import Commander, CommanderParams
from ai_interface import Bark, TTSParams
from utils import ROSPublisher, RobotChat

from pathlib import Path
from typing import Optional

from rclpy.action import ActionServer
from rclpy.node import Node


class CommanderActionServerInterface(Node):

    def __init__(self, name: str):
        super().__init__(name)

        self.declare_parameter('type', 'CHAT')

        self.declare_parameter('speech_to_text.host', '')
        self.declare_parameter('speech_to_text.api_key', '')
        self.declare_parameter('speech_to_text.name', '')

        self.declare_parameter('language_model.host', '')
        self.declare_parameter('language_model.api_key', '')
        self.declare_parameter('language_model.name', '')
        self.declare_parameter('language_model.grammar_file', '')
        self.declare_parameter('language_model.initial_prompt_file', '')
        self.declare_parameter('language_model.initial_prompt_context', '')

        self.declare_parameter('text_to_speech.host', '')
        self.declare_parameter('text_to_speech.api_key', '')
        self.declare_parameter('text_to_speech.name', '')
        self.declare_parameter('text_to_speech.voice_type', 'announcer')
        self.declare_parameter('text_to_speech.pytorch_model_path', '')
        self.declare_parameter('text_to_speech.use_pytorch', False)

        self.commander: Optional[Commander] = None
        self.pytorchTTS: Optional[Bark] = None

    def initialize(self):
        grammar: str = self.get_parameter('language_model_grammar_file').get_parameter_value().string_value
        if grammar and not Path(grammar).is_file():
            self.get_logger().warning(f"Grammar file '{grammar}' not found, will not use grammar constraints.")
            grammar = ""

        self.commander = Commander(
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
