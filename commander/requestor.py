from typing import Any

import requests


class Requestor:
    def __init__(self, url: str) -> None:
        self.url: str = url

    def post(self, prompt: str) -> Any:
        headers = {"Authorization": "Bearer no-key",
                   "Content-Type": "application/json"}
        payload = [{"role": "user", "content": prompt}]

        return requests.post(self.url + "/v1/chat/completions", data=payload, headers=headers)
