from dataclasses import dataclass
from typing import Optional, Any
from pynput.keyboard import Key

import os
import openai


@dataclass
class OpenAIParams:
    chat_model: str = "gpt-3.5-turbo"
    voice_model: str = "whisper-1"
    image_size: str = "256x256"
    initial_chat_prompt: Optional[str] = None
    api_key: Optional[str] = None


class OpenAI:
    def __init__(self, params: OpenAIParams) -> None:
        if params.api_key is not None: openai.api_key = params.api_key

        self.chat_model: str = params.chat_model
        self.voice_model: str = params.voice_model
        self.image_size: str = params.image_size

        self.init_prompt: str = params.initial_chat_prompt if params.initial_chat_prompt is not None else ""

    def get_voice_prompt(self, key: Key = Key.f10) -> str:
        from utils import Recorder
        # FIXME(recording): make it inline instead of this crappy file saving
        with Recorder() as r:
            r.hold_to_record(key)
            r.save_recording(filename="prompt.wav")
        return openai.Audio.translate(model=self.voice_model, file=open("prompt.wav", "rb")).text

    def get_messages(self, prompt: str) -> str:
        messages = [{"role": "system", "content": self.init_prompt},
                    {"role": "user", "content": prompt}]
        response = openai.ChatCompletion.create(model=self.chat_model, messages=messages, temperature=0.7)
        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f'raw response:\n{response.choices[0].message["content"]}')
        return response.choices[0].message["content"]

    def generate_image(self, prompt: str, filename: str = "image.png"):
        import requests
        import shutil

        url = openai.Image.create(prompt=prompt, n=1, size=self.image_size)['data'][0]['url']
        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"{url}")

        r = requests.get(url, stream=True)
        if r.status_code == 200:
            r.raw.decode_content = True
            with open(filename, 'wb') as f:
                shutil.copyfileobj(r.raw, f)
            print(f"Image '{filename}' downloaded.")
        else:
            print("Image couldn't be retrieved.")
