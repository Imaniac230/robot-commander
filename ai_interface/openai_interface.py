import os

import openai
import json
import glob


def generate_image(prompt: str, filename: str = "image.png"):
    import requests
    import shutil

    url = openai.Image.create(prompt=prompt, n=1, size='256x256')['data'][0]['url']
    if os.getenv("DEBUG") is not None:
        print(f"{url}")

    r = requests.get(url, stream=True)
    if r.status_code == 200:
        r.raw.decode_content = True
        with open(filename, 'wb') as f:
            shutil.copyfileobj(r.raw, f)

        print(f"Image '{filename}' downloaded.")
    else:
        print(f"Image couldn't be retrieved.")


class OpenAI:
    def __init__(self, key: str = None, context: list = None) -> None:
        if key is not None:
            openai.api_key = key
        self.chat_model = "gpt-3.5-turbo"
        self.voice_model = "whisper-1"

        self.messages = ""
        for file in glob.glob("messages/*.txt"):
            self.messages += "<" + os.path.basename(file).split('.')[0] + ">"
            with open(file, 'r') as rd:
                self.messages += rd.read()
            self.messages += "</" + os.path.basename(file).split('.')[0] + ">" + "\n"

        self.context = ""
        for item in context:
            self.context += item + "\n"

        if os.getenv("DEBUG") is not None:
            if context is not None:
                print(f'context:\n{self.context}')
            print(f'messages:\n{self.messages}')

        self.init_prompt = ""
        if context is not None:
            self.init_prompt += f'''
                The following lines provide context for mapping any positions and orientations to keywords:
                {self.context}'''
        self.init_prompt += f'''
            Now, following are the actual format of the messages with their name as the tags <name></name>:
            {self.messages}

            Return a python list of these messages required in order to achieve the user's goals in ROS2 without any 
            explanation. Goals will be delimited by the <prompt> tags. Do not append the names of the messages in the 
            list. All the properties of the messages should be enclosed within double quotes. Each message represents 
            1 second of the goal done, so add messages for every second to the list according to the goals. Make sure 
            you actually wrap the messages inside a python list.'''

    def get_voice_prompt(self, seconds: int = 6):
        from utils import Recorder
        # FIXME(recording): make it inline instead of this crappy file saving
        with Recorder() as r:
            r.record_for(seconds)
            r.save_recording(filename="prompt.wav")
        return openai.Audio.translate(model=self.voice_model, file=open("prompt.wav", "rb")).text

    def get_messages(self, prompt: str) -> str:
        messages = [{"role": "system", "content": self.init_prompt},
                    {"role": "user", "content": f"<prompt>{prompt}<prompt>"}]
        response = openai.ChatCompletion.create(model=self.chat_model, messages=messages, temperature=0.7)
        if os.getenv("DEBUG") is not None:
            print(f'raw response:\n{response.choices[0].message["content"]}')
        res = json.loads(response.choices[0].message["content"])
        return res
