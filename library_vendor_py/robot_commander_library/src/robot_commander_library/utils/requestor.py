from typing import Optional, Dict, Iterator, Tuple, Any, BinaryIO, List
from dataclasses import dataclass
import requests as rq
import json as j


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
        # NOTE: do not specify {"Content-Type": "multipart/form-data"}, must be provided by requests together with the boundary
        self.headers.pop("Content-Type", None)
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, files=file, data=data, headers=self.headers))
        return r.json() if r is not None else r

    # TODO(restricted-type): unsupported operand type(s) for |: '_GenericAlias' and '_GenericAlias'
    #   Dict[str, List[str] | List[Dict[str, str]] | Dict[str, str] | str]
    def respond(self, endpoint: str, json: Dict) -> Optional[Dict]:
        self.headers["Content-Type"] = "application/json"
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=json, headers=self.headers))
        return r.json() if r is not None else r

    def synthesize(self, endpoint: str, json: Dict[str, str]) -> Optional[Iterator]:
        self.headers["Content-Type"] = "application/json"
        r: Optional[rq.Response] = self.validate_response(rq.post(self.url + endpoint, json=json, headers=self.headers))
        return r.iter_content() if r is not None else r


@dataclass
class HostProviderParams:
    stt_host: str
    llm_host: str
    tts_host: Optional[str] = None
    stt_api_key: Optional[str] = None  # required by external APIs
    llm_api_key: Optional[str] = None  # required by external APIs
    tts_api_key: Optional[str] = None  # required by external APIs
    stt_name: Optional[str] = None  # required by external APIs
    llm_name: Optional[str] = None  # required by external APIs
    tts_name: Optional[str] = None  # required by external APIs
    llm_system_prompt: Optional[str] = None  # should be used if using external APIs
    llm_json_schema_file: Optional[str] = None
    tts_voice: Optional[str] = None


class HostProvider:
    def __init__(self, params: HostProviderParams) -> None:
        self.supported_providers: List[str] = ["openai", "anthropic"]

        if not params.stt_host: raise ValueError("Speech-to-text host not provided.")
        if not params.llm_host: raise ValueError("Language-model host not provided.")

        if any(service in params.stt_host.lower() for service in self.supported_providers):
            if params.stt_api_key is None or not params.stt_api_key: raise ValueError(f"External speech-to-text host '{params.stt_host}' requires an API key, but it was not specified.")
            if params.stt_name is None or not params.stt_name: raise ValueError(f"External speech-to-text host '{params.stt_host}' requires a model name, but it was not specified.")
        if any(service in params.llm_host.lower() for service in self.supported_providers):
            if params.llm_api_key is None or not params.llm_api_key: raise ValueError(f"External language-model host '{params.llm_host}' requires an API key, but it was not specified.")
            if params.llm_name is None or not params.llm_name: raise ValueError(f"External language-model host '{params.llm_host}' requires a model name, but it was not specified.")

        self.stt_endpoint: str = "/inference"  # NOTE: default for local
        if "openai" in params.stt_host.lower(): self.stt_endpoint = "/v1/audio/translations"

        self.llm_endpoint: str = "/v1/chat/completions"  # NOTE: default for local
        if "openai" in params.llm_host.lower():
            self.llm_endpoint = "/v1/chat/completions"
        elif "anthropic" in params.llm_host.lower():
            self.llm_endpoint = "/v1/messages"

        self.tts_endpoint: str = ""
        if params.tts_host is not None and params.tts_host:
            if params.tts_voice is None or not params.tts_voice: raise ValueError("Text-to-speech voice type not provided.")
            if any(service in params.tts_host.lower() for service in self.supported_providers):
                if params.tts_api_key is None or not params.tts_api_key: raise ValueError(f"External text-to-speech host '{params.tts_host}' requires an API key, but it was not specified.")
                if params.tts_name is None or not params.tts_name: raise ValueError(f"External text-to-speech host '{params.tts_host}' requires a model name, but it was not specified.")

            # TODO(tts-server): local tts server is only experimental for now
            self.tts_endpoint = "/v1/audio/speech"  # NOTE: default for local
            if "openai" in params.tts_host.lower():
                self.tts_endpoint = "/v1/audio/speech"

        self.params: HostProviderParams = params

    def stt_payload(self, audio_file: str) -> Tuple[Dict[str, Tuple[str, BinaryIO, str]], Optional[Dict[str, str]]]:
        if "openai" in self.params.stt_host.lower():
            file_payload = {"file": (audio_file, open(audio_file, mode="rb"), "audio/x-wav")}
            data_payload = {"model": self.params.stt_name}
            return file_payload, data_payload

        # NOTE: default for local
        file_payload = {"file": (audio_file, open(audio_file, mode="rb"), "audio/x-wav")}
        data_payload = {"model": self.params.stt_name} if self.params.stt_name is not None else None
        return file_payload, data_payload

    # TODO(restricted-type): unsupported operand type(s) for |: '_GenericAlias' and '_GenericAlias'
    #   Dict[str, List[str] | List[Dict[str, str]] | Dict[str, str] | str]
    def llm_payload(self, input_string: str, stop_strings: List[str]) -> Dict:
        if "openai" in self.params.llm_host.lower():
            json_payload = {"messages": [{"role": "user", "content": input_string}], "stop": stop_strings, "model": self.params.llm_name}
            if self.params.llm_system_prompt is not None: json_payload["messages"] = [{"role": "system", "content": self.params.llm_system_prompt}, json_payload["messages"][0]]
            if self.params.llm_json_schema_file is not None:
                # with open(self.params.llm_json_schema_file, 'r') as sch: json_payload["response_format"] = {"type": "json_object", "schema": j.load(sch)}
                # NOTE: API change -> the 'json_object' with 'schema' usage is not supported anymore
                with open(self.params.llm_json_schema_file, 'r') as sch: schema = j.load(sch)
                # FIXME(external-schema): API allows only "object" type in the schema root object,
                #   does not allow for missing "properties" key -> this will currently NOT be interpreted correctly when parsing the response
                schema = {"type": "object", "properties": {"data": schema}}
                json_payload["response_format"] = {"type": "json_schema", "json_schema": {"name": "custom_schema", "schema": schema}}
            return json_payload

        if "anthropic" in self.params.llm_host.lower():
            json_payload = {"messages": [{"role": "user", "content": input_string}], "stop_sequences": stop_strings, "model": self.params.llm_name}
            if self.params.llm_system_prompt is not None: json_payload["system"] = self.params.llm_system_prompt
            if self.params.llm_json_schema_file is not None:
                with open(self.params.llm_json_schema_file, 'r') as sch: schema = j.load(sch)
                # TODO(external-schema): API allows only "object" type in the schema root object,
                #   seems to interpret the schema correctly even if the remaining definition is still actually for an "array"
                schema["type"] = "object"
                json_payload["tools"] = [{"name": "custom_schema", "description": "", "input_schema": schema}]
            # FIXME(max-tokens): this is required for anthropic and should be unified with local and openai calls
            json_payload["max_tokens"] = 1042
            return json_payload

        # NOTE: default for local
        json_payload = {"messages": [{"role": "user", "content": input_string}], "stop": stop_strings}
        if self.params.llm_name is not None: json_payload["model"] = self.params.llm_name
        if self.params.llm_system_prompt is not None: json_payload["messages"] = [{"role": "system", "content": self.params.llm_system_prompt}, json_payload["messages"][0]]
        if self.params.llm_json_schema_file is not None:
            with open(self.params.llm_json_schema_file, 'r') as sch: json_payload["response_format"] = {"type": "json_object", "schema": j.load(sch)}
        return json_payload

    def tts_payload(self, input_string: str) -> Dict[str, str]:
        if "openai" in self.params.tts_host.lower():
            json_payload = {"input": input_string, "voice": self.params.tts_voice, "response_format": "wav", "model": self.params.tts_name}
            return json_payload

        # TODO(tts-server): local tts server is only experimental for now
        # NOTE: default for local
        json_payload = {"input": input_string}
        return json_payload

    def transcription(self, result: Optional[Dict]) -> Optional[str]:
        if result is None: return result

        if "openai" in self.params.stt_host.lower(): return result["text"]

        # NOTE: default for local
        return result["text"]

    def response(self, result: Optional[Dict]) -> Optional[str]:
        if result is None: return result

        # FIXME(external-schema): this will currently NOT interpret the schema-constrained response correctly
        if "openai" in self.params.llm_host.lower(): return result["choices"][0]["message"]["content"]
        if "anthropic" in self.params.llm_host.lower():
            # NOTE: outputs all json data with single-quoted keys, but the json parser we use down the line requires strictly double-quotes
            if len(result["content"]) > 1 and result["content"][1]["type"] == "tool_use": return str(result["content"][1]["input"]["items"]).replace("'", '"')
            return result["content"][0]["text"]

        # NOTE: default for local
        return result["choices"][0]["message"]["content"]

    @classmethod
    def synthesis(cls, result: Optional[Iterator]) -> Optional[Any]:
        # NOTE: we're currently not parsing the result in any way
        return result
