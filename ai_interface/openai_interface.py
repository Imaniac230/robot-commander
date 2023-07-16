import os

import openai
import json
import glob


def record_audio(seconds: int, filename: str = "output.wav"):
    import pyaudio
    import wave

    channels = 2
    chunk = 1024  # Record in chunks of 1024 samples
    sample_format = pyaudio.paInt16  # 16 bits per sample
    fs = 44100  # Record at 44100 samples per second

    p = pyaudio.PyAudio()
    stream = p.open(format=sample_format,
                    channels=channels,
                    rate=fs,
                    frames_per_buffer=chunk,
                    input=True)
    frames = [stream.read(chunk) for _ in range(0, int(fs / chunk * seconds))]
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(sample_format))
    wf.setframerate(fs)
    wf.writeframes(b''.join(frames))
    wf.close()


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


class OpenAIInterface:
    def __init__(self, key: str = None) -> None:
        if key is not None:
            openai.api_key = key
        self.chat_model = "gpt-3.5-turbo"
        self.voice_model = "whisper-1"

        self.messages = ""
        for file in glob.glob("messages/*.txt"):
            self.messages += "<" + os.path.basename(file).split('.')[0] + ">"
            with open(file, 'r') as rd:
                self.messages += rd.read()
            self.messages += "<" + os.path.basename(file).split('.')[0] + ">" + "\n"

        if os.getenv("DEBUG") is not None:
            print(f'messages:\n{self.messages}')

        self.init_prompt = f'''
            Following are the format of the messages with their name as the tags.
            {self.messages}

            Return a python list of these messages required in order to achieve the user's goals in ROS2 without any 
            explanation. Goals will be delimited by the <prompt> tags. Do not append the names of the messages in the 
            list. All the properties of the messages should be enclosed within double quotes. Each message represents 
            1 second of the goal done, so add messages for every second to the list according to the goals. Make sure 
            you actually wrap the messages inside a python list.'''

    def get_voice_prompt(self, seconds: int = 6):
        # FIXME(recording): make it inline instead of this crappy file saving
        record_audio(seconds, filename="prompt.wav")
        return openai.Audio.translate(model=self.voice_model, file=open("prompt.wav", "rb")).text

    def get_messages(self, prompt: str) -> str:
        messages = [{"role": "system", "content": self.init_prompt},
                    {"role": "user", "content": f"<prompt>{prompt}<prompt>"}]
        response = openai.ChatCompletion.create(model=self.chat_model, messages=messages, temperature=0.7)
        if os.getenv("DEBUG") is not None:
            print(f'raw response:\n{response.choices[0].message["content"]}')
        res = json.loads(response.choices[0].message["content"])
        return res
