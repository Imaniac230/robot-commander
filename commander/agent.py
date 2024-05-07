import ai_interface as ai
from typing import Optional
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
    
    def respond(self): raise NotImplementedError("implement respond for agent")
