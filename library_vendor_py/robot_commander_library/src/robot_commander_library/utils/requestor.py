from typing import Optional, Dict, Iterator, Tuple, Any, BinaryIO, List
import requests as rq


class Requestor:
    def __init__(self, url: str, api_key: Optional[str] = None) -> None:
        self.headers: Dict[str, str] = dict()
        if api_key is not None:
            # This is used by the OpenAI API
            self.headers["Authorization"] = f"Bearer {api_key}"
            # This is used by the Anthropic API
            self.headers["X-API-Key"] = api_key
            self.headers["Anthropic-Version"] = "2023-06-01"
        self.url: str = url

    @classmethod
    def validate_response(cls, r: rq.Response) -> Optional[rq.Response]:
        if not r.ok:
            print(f"Failed to get response from server (reason: {r.reason}, code: {r.status_code}), message:\n{r.text}")
            return None
        if r.headers["Content-Type"] == "application/json" and "error" in r.json():
            print(f"Server failed to process request, message:\n{r.text}")
            return None
        return r

    def transcribe(self, endpoint: str, file: Dict[str, Tuple[Any, BinaryIO, str]], data: Optional[Dict[str, str]] = None) -> Optional[Dict]:
        #NOTE: do not specify {"Content-Type": "multipart/form-data"}, must be provided by requests together with the boundary
        self.headers.pop("Content-Type", None)
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, files=file, data=data, headers=self.headers))
        return r.json() if r is not None else r

    #TODO(restricted-type): incompatible operator | for alias type?
    def respond(self, endpoint: str, json: Dict) -> Optional[Dict]:
        self.headers["Content-Type"] = "application/json"
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=json, headers=self.headers))
        return r.json() if r is not None else r

    def synthesize(self, endpoint: str, json: Dict[str, str]) -> Optional[Iterator]:
        self.headers["Content-Type"] = "application/json"
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=json, headers=self.headers))
        return r.iter_content() if r is not None else r
