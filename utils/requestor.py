from typing import Optional, Dict, Iterator
import requests
import json


class Requestor:
    def __init__(self, url: str, api_key: Optional[str] = None) -> None:
        self.headers: Dict[str, str] = {"Authorization": f"Bearer {api_key if api_key is not None else 'no-key'}"}
        self.url: str = url


    def post_chat(self, prompt: str) -> Optional[Dict]:
        self.headers += {"Content-Type": "application/json"}

        payload = {"model": "gpt-4o", "messages": [{"role": "user", "content": prompt}]}

        response = requests.post(self.url + "/v1/chat/completions", json=payload, headers=self.headers)
        if not response.ok:
            print(f"Failed to get response from server (reason: {response.reason}, code: {response.status_code}),"
                  f" message: {json.loads(response.text)['error']['message']})")
            return None

        return response.json()

    def post_audio(self, prompt: str, voice: str) -> Optional[Iterator]:
        self.headers += {"Content-Type": "application/json"}

        payload = {"model": "tts-1", "input": prompt, "voice": voice, "response_format": "wav"}

        response = requests.post(self.url + "/v1/audio/speech", json=payload, headers=self.headers)
        if not response.ok:
            print(f"Failed to get response from server (reason: {response.reason}, code: {response.status_code}),"
                  f" message: {json.loads(response.text)['error']['message']}")
            return None

        return response.iter_content()

    def post_translation(self, file: str) -> Optional[Dict]:
        #NOTE: do not specify {"Content-Type": "multipart/form-data"}, must be provided by requests together with the boundary

        payload = {"file": (file, open(file, mode="rb"), "audio/x-wav")}
        data = {"model": "whisper-1"}

        response = requests.post(self.url + "/v1/audio/translations", files=payload, data=data, headers=self.headers)
        print(f"{response.request.body}")
        if not response.ok:
            print(f"Failed to get response from server (reason: {response.reason}, code: {response.status_code},"
                  f" message: {json.loads(response.text)['error']['message']})")
            return None

        return response.json()
