from typing import Optional, Dict, Iterator, Tuple
import requests as rq
import json


class Requestor:
    def __init__(self, url: str, api_key: Optional[str] = None) -> None:
        self.headers: Dict[str, str] = dict(Authorization=f"Bearer {api_key if api_key is not None else 'no-key'}")
        self.url: str = url

    @classmethod
    def validate_response(cls, r: rq.Response) -> Optional[rq.Response]:
        if not r.ok:
            print(f"Failed to get response from server (reason: {r.reason}, code: {r.status_code}),"
                  f" message: {json.loads(r.text)['error']['message']})")
            return None
        return r

    def transcribe(self, endpoint: str, file: str) -> Optional[Dict]:
        #NOTE: do not specify {"Content-Type": "multipart/form-data"}, must be provided by requests together with the boundary
        self.headers.pop("Content-Type", None)
        payload = {"file": (file, open(file, mode="rb"), "audio/x-wav")}
        data = {"model": "whisper-1"}

        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, files=payload, data=data, headers=self.headers))
        return r.json() if r is not None else r


    def respond(self, endpoint: str, prompt: str) -> Optional[Dict]:
        self.headers["Content-Type"] = "application/json"
        payload = {"model": "gpt-4o", "messages": [{"role": "user", "content": prompt}]}

        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=payload, headers=self.headers))
        return r.json() if r is not None else r


    def synthesize(self, endpoint: str, prompt: Dict[str, str]) -> Optional[Iterator]:
        self.headers["Content-Type"] = "application/json"
        payload = {"model": "tts-1", "input": prompt["prompt"], "voice": prompt["voice"], "response_format": "wav"}

        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=payload, headers=self.headers))
        return r.iter_content() if r is not None else r
