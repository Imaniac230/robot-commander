from dataclasses import dataclass
from typing import Optional, Any
from pynput.keyboard import Key

import os
import openai


@dataclass
class OpenAIParams:
    chat_model: str = "gpt-3.5-turbo"
    voice_model: str = "whisper-1"
    speech_model: str = "tts-1"
    image_model: str = "dall-e-3"

    image_size: str = "1024x1024"
    speech_voice: str = "alloy"
    chat_initial_prompt: Optional[str] = None
    api_key: Optional[str] = None


class OpenAI:
    def __init__(self, params: OpenAIParams) -> None:
        if params.api_key is not None: openai.api_key = params.api_key
        self.client = openai.OpenAI()

        self.chat_model: str = params.chat_model
        self.voice_model: str = params.voice_model
        self.speech_model: str = params.speech_model
        self.image_model: str = params.image_model

        self.init_prompt: str = params.chat_initial_prompt if params.chat_initial_prompt is not None else ""
        self.image_size: str = params.image_size
        self.speech_voice: str = params.speech_voice

    def transcribe(self, key: Key = Key.f10) -> str:
        from utils import Recorder
        # FIXME(recording): make it inline instead of this crappy file saving
        with Recorder() as r:
            r.hold_to_record(key)
            r.save_recording(filename="prompt.wav")
        return self.client.audio.translations.create(model=self.voice_model, file=open("prompt.wav", "rb")).text

    def respond(self, prompt: str) -> str:
        messages = [{"role": "system", "content": self.init_prompt},
                    {"role": "user", "content": prompt}]
        response = self.client.chat.completions.create(model=self.chat_model, messages=messages, temperature=0.7)
        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f'raw response:\n{response.choices[0].message.content}')
        return response.choices[0].message.content

    def generate_image(self, prompt: str, filename: str = "image.png") -> None:
        import requests
        import shutil

        url = openai.images.generate(model=self.image_model, prompt=prompt, n=1, size=eval("self.image_size")).data[0].url
        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"{url}")

        r = requests.get(url, stream=True)
        if r.status_code == 200:
            r.raw.decode_content = True
            with open(filename, 'wb') as f:
                shutil.copyfileobj(r.raw, f)
            print(f"\nImage '{filename}' downloaded.")
        else:
            print("\nImage couldn't be retrieved.")

    def synthesize(self, prompt: str, filename: str = "speech.wav") -> Any:
        import sounddevice as sd
        import soundfile as sf

        response = openai.audio.speech.create(model=self.speech_model, voice=eval("self.speech_voice"), input=prompt, response_format="wav")
        response.write_to_file(filename)
        # FIXME(playback): make it inline instead of this crappy file saving
        data, rate = sf.read(filename)
        sd.wait()
        sd.play(data, rate, blocking=False)

        return response.content
