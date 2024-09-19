from typing import Optional
from pathlib import Path
import glob
import os


class SystemPrompt:
    def __init__(self, file_path: str) -> None:
        with open(file_path, 'r') as f: self._prompt = f.read()

    def prompt(self) -> str: return self._prompt


class RobotChat(SystemPrompt):
    def __init__(self, file_path: str, personality: Optional[str] = None) -> None:
        super().__init__(file_path)
        if personality is not None:
            if int(os.getenv("DEBUG", "0")) >= 1: print(f"personality:\n{personality}")

            self._prompt += f"Here is some more info on your personality and character traits:\n{personality}\n\n"

        if int(os.getenv("DEBUG", "0")) >= 2: print(f"full system prompt:\n{self._prompt}")


class ROSPublisher(SystemPrompt):
    def __init__(self, file_path: str, messages_directory: str, environment: Optional[str] = None) -> None:
        super().__init__(file_path)
        messages = ""
        # TODO(rag-prompts): the pose contexts should eventually be ported out into the PromptContext class
        contexts = ""
        if messages_directory and Path(messages_directory).is_dir():
            for file in glob.glob(str(Path(messages_directory).resolve()) + "/*.json"):
                messages += "<" + os.path.basename(file).split('.')[0] + ">\n"
                with open(file, 'r') as rd: messages += rd.read()
                messages += "\n</" + os.path.basename(file).split('.')[0] + ">\n"

            for file in glob.glob(str(Path(messages_directory).resolve()) + "/contexts/*.txt"):
                with open(file, 'r') as rd: contexts += rd.read() + "\n"

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"messages:\n{messages}")
            if contexts: print(f"context:\n{contexts}")

        self._prompt += 'Details about the ROS2 messages that you will output:\n'
        self._prompt += f'The following shows the ROS2 messages in the required JSON format:\n{messages}\n'
        self._prompt += 'Each message is enclosed inside a tag that defines its ROS2 name (<message-name></message-name>).\n\n'
        if contexts:
            self._prompt += f'The following lines provide context for mapping any received keywords to corresponding properties of the JSON response:\n{contexts}\n'
            self._prompt += 'Make sure you always match all JSON property values exactly each time you detect a corresponding keyword.\n\n'
        if environment is not None:
            self._prompt += f'Additional properties of the environment that you are operating in:\n{environment}\n\n'

        if int(os.getenv("DEBUG", "0")) >= 2: print(f"full system prompt:\n{self._prompt}")


class PromptContext:
    def __init__(self, data: str) -> None:
        self._context = data

    def context(self) -> str: return self._context


class ROSPublisherContext(PromptContext):
    def __init__(self, data: str) -> None:
        super().__init__(data)
        self._context = f'Your current state is:\n{self._context}\n'
        self._context += 'Make sure to use this information when processing the next command.\n\n'

        if int(os.getenv("DEBUG", "0")) >= 1: print(f"prompt context:\n{self._context}")
