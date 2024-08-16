from typing import Optional, Any
from typing_extensions import Self
from enum import Enum, unique
from robot_commander_library.utils import Requestor, HostProvider

import robot_commander_library.ai_interface as ai
import sounddevice as sd
import soundfile as sf
import os


class Agent:
    def __init__(self, stt: ai.STT, llm: ai.LLM, tts: Optional[ai.TTS] = None) -> None:
        # TODO(agent): currently keeping only the tts model as optional,
        # decide what the general interface should be
        self.stt: ai.STT = stt
        self.llm: ai.LLM = llm
        self.tts: Optional[ai.TTS] = tts

    def launch(self) -> Self:
        self.stt.start_server()
        self.llm.start_server()
        if self.tts is not None: self.tts.start_server()
        return self

    def respond(self, audio_file: str, load_models: bool = False, playback_response: bool = False) -> Any:
        text_response: str = self.llm.respond(self.stt.transcribe(audio_file, load_model=load_models), inline_response=True if self.tts is not None else False, load_model=load_models)
        if self.tts is None: return text_response

        voice_response: Any = self.tts.synthesize(text_response, load_model=load_models)
        if playback_response: self.tts.play_synthesis()
        return voice_response

@unique
class CommanderState(Enum):
    UNKNOWN = 0
    TRANSCRIBING = 1
    RESPONDING = 2
    SYNTHESISING = 3
    PLAYING_RESPONSE = 4
    IDLE = 5
    ERROR = -1

class Commander:
    def __init__(self, api: HostProvider, tts_generated_file: str = "output_response.wav") -> None:
        self.request_specifier: str = "REQUEST:"
        self.state: CommanderState = CommanderState.UNKNOWN
        self.api: HostProvider = api

        self.last_transcription: str = ""
        self.last_response: str = ""
        self.last_synthesis: Any = None

        self.tts_generated_file: str = tts_generated_file

    def respond(self, audio_file: str, playback_response: bool = False) -> Any:
        self.state = CommanderState.TRANSCRIBING
        stt_payload = self.api.stt_payload(audio_file)
        transcription = self.api.transcription(Requestor(self.api.params.stt_host, api_key=self.api.params.stt_api_key).transcribe(self.api.stt_endpoint, stt_payload[0], stt_payload[1]))
        if transcription is None:
            print("Failed to get transcription from STT.")
            self.last_transcription = ""
            self.state = CommanderState.ERROR
            return None
        self.last_transcription = transcription

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven stt prompt file:\n{audio_file}\n")
            print(f"\nreturned stt transcription:\n{self.last_transcription}\n")

        self.state = CommanderState.RESPONDING
        llm_input: str = self.request_specifier + "\n" + self.last_transcription
        llm_payload = self.api.llm_payload(llm_input, stop_strings=[self.request_specifier])
        response = self.api.response(Requestor(self.api.params.llm_host, api_key=self.api.params.llm_api_key).respond(self.api.llm_endpoint, llm_payload))
        if response is None:
            print("Failed to get response from LLM.")
            self.last_response = ""
            self.state = CommanderState.ERROR
            return None
        self.last_response = response

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven llm prompt:\n{llm_input}\n")
            print(f"\nreturned llm response:\n{self.last_response}\n")

        if self.api.params.tts_host is None:
            self.state = CommanderState.IDLE
            return self.last_response
        self.state = CommanderState.SYNTHESISING
        tts_payload = self.api.tts_payload(self.last_response)
        synthesis = self.api.synthesis(Requestor(self.api.params.tts_host, api_key=self.api.params.tts_api_key).synthesize(self.api.tts_endpoint, tts_payload))
        if synthesis is None:
            print("Failed to get synthesis from TTS.")
            self.last_synthesis = None
            self.state = CommanderState.ERROR
            return None
        self.last_synthesis = synthesis
        with open(self.tts_generated_file, mode="wb") as f:
            for data in self.last_synthesis: f.write(data)

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven tts prompt:\n{self.last_response}\n")
            print(f"\nreturned tts synthesis file:\n{self.tts_generated_file}\n")

        if playback_response:
            self.state = CommanderState.PLAYING_RESPONSE
            data, rate = sf.read(self.tts_generated_file)
            sd.wait()
            sd.play(data, rate, blocking=False)
        self.state = CommanderState.IDLE
        return self.last_synthesis
