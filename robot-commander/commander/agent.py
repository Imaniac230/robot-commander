from typing import Optional, Any
from typing_extensions import Self
from dataclasses import dataclass
from utils import Requestor

import ai_interface as ai
import sounddevice as sd
import soundfile as sf
import json
import os


class Agent:
    def __init__(self, stt: ai.STT, llm: ai.LLM, tts: Optional[ai.TTS] = None) -> None:
        #TODO(agent): currently keeping only the tts model as optional,
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

#TODO(ros): this example should be implemented as the main ROS node for this project and it shouldn't have to be limited only to python in any way,
#   also expose all relevant parameters into the config
@dataclass
class CommanderParams:
    stt_host: str
    stt_endpoint: str
    llm_host: str
    llm_endpoint: str
    tts_host: Optional[str] = None
    tts_endpoint: Optional[str] = None
    tts_voice: Optional[str] = None
    stt_name: Optional[str] = None # currently valid only for the openai API
    llm_name: Optional[str] = None # currently valid only for the openai API
    tts_name: Optional[str] = None # currently valid only for the openai API
    tts_generated_file: str = "output_response.wav"
    api_key: Optional[str] = None # currently valid only for the openai API

class Commander:
    def __init__(self, params: CommanderParams) -> None:
        self.params = params

        self.last_transcription: str = ""
        self.last_response: str = ""
        self.last_synthesis: Any = None

    def respond(self, audio_file: str, playback_response: bool = False, response_format: Optional[str] = None, system_prompt: Optional[str] = None) -> Any:
        file_payload = {"file": (audio_file, open(audio_file, mode="rb"), "audio/x-wav")}
        data_payload = {"model": self.params.stt_name} if self.params.stt_name is not None else None
        transcription = Requestor(self.params.stt_host, api_key=self.params.api_key).transcribe(self.params.stt_endpoint, file_payload, data_payload)
        if transcription is None:
            print("Failed to get transcription from STT.")
            self.last_transcription = ""
            return None
        self.last_transcription = transcription['text']

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven stt prompt file:\n{audio_file}\n")
            print(f"\nreturned stt transcription:\n{self.last_transcription}\n")

        json_payload = {"messages": [{"role": "user", "content": "REQUEST:\n" + self.last_transcription}], "stop": ["REQUEST:"]}
        if system_prompt is not None: json_payload["messages"] = [{"role": "system", "content": system_prompt}, json_payload["messages"][0]]
        if response_format is not None:
            with open(response_format, 'r') as sch: json_payload["response_format"] = {"type": "json_object", "schema": json.load(sch)}
        if self.params.llm_name is not None: json_payload["model"] = self.params.llm_name
        response = Requestor(self.params.llm_host, api_key=self.params.api_key).respond(self.params.llm_endpoint, json_payload)
        if response is None:
            print("Failed to get response from LLM.")
            self.last_response = ""
            return None
        self.last_response = response['choices'][0]['message']['content']

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven llm prompt:\n{json_payload['messages'][0 if system_prompt is None else 1]['content']}\n")
            print(f"\nreturned llm response:\n{self.last_response}\n")

        if self.params.tts_host is None or self.params.tts_endpoint is None: return self.last_response
        if self.params.tts_voice is None: raise ValueError("TTS voice type not provided.")
        #FIXME(tts-server): this only supports oai API, test with bark_cpp server once ready
        json_payload = {"input": self.last_response, "voice": self.params.tts_voice, "response_format": "wav"}
        if self.params.tts_name is not None: json_payload["model"] = self.params.tts_name
        synthesis = Requestor(self.params.tts_host, api_key=self.params.api_key).synthesize(self.params.tts_endpoint, json_payload)
        if synthesis is None:
            print("Failed to get synthesis from TTS.")
            self.last_synthesis = None
            return None
        self.last_synthesis = synthesis
        with open(self.params.tts_generated_file, mode="wb") as f:
            for data in self.last_synthesis: f.write(data)

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"\ngiven tts prompt:\n{json_payload['input']}\n")
            print(f"\nreturned tts synthesis file:\n{self.params.tts_generated_file}\n")

        if playback_response:
            data, rate = sf.read(self.params.tts_generated_file)
            sd.wait()
            sd.play(data, rate, blocking=False)
        return self.last_synthesis
