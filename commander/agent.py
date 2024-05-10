import ai_interface as ai
from typing import Optional, Any
from typing_extensions import Self

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
    
    def respond(self, audio_file: str) -> Any:
        text_response: str = self.llm.respond(self.stt.transcribe(audio_file))
        if self.tts is None: return text_response

        voice_response: Any = self.tts.synthesize(text_response)
        self.tts.play_synthesis()
        return voice_response
