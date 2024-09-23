from typing import Optional
from pathlib import Path
import glob
import os


class SystemPrompt:
    def __init__(self, file_path: str, *args, **kwargs) -> None:
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
        if messages_directory and Path(messages_directory).is_dir():
            for file in glob.glob(str(Path(messages_directory).resolve()) + "/*.json"):
                messages += "<" + os.path.basename(file).split('.')[0] + ">\n"
                with open(file, 'r') as rd: messages += rd.read()
                messages += "\n</" + os.path.basename(file).split('.')[0] + ">\n"

        if int(os.getenv("DEBUG", "0")) >= 1: print(f"messages:\n{messages}")

        self._prompt += 'Details about the ROS2 messages that you will output:\n'
        self._prompt += f'The following shows all supported ROS2 messages in the required JSON format:\n{messages}\n'
        self._prompt += 'Each message is enclosed inside a tag that defines its ROS2 name (<message-name></message-name>).\n\n'
        if environment is not None:
            self._prompt += f'Additional properties of the environment that you are operating in:\n{environment}\n\n'

        if int(os.getenv("DEBUG", "0")) >= 2: print(f"full system prompt:\n{self._prompt}")


class PromptContext:
    def __init__(self, *args, **kwargs) -> None:
        self._context = ""

    def context(self) -> str: return self._context


class PoseMessageContext(PromptContext):
    def __init__(self, data: str, contexts: str) -> None:
        super().__init__()

        if int(os.getenv("DEBUG", "0")) >= 1:
            print(f"context:\n{data}")
            if contexts: print(f"additional contexts:\n{contexts}")

        if contexts:
            self._context += f'The following lines provide context for mapping any received keywords to corresponding properties of the JSON response:\n{contexts}\n'
            self._context += 'Make sure you always match all JSON property values exactly each time the task relates to the corresponding keyword.\n\n'

        self._context += f'Your current body frame context is:\n{data}\n'
        self._context += 'The following lines provide additional context for assigning values to the "position" JSON property, when the task is not related to any keyword:\n'
        # TODO(message-context): this is currently only for translational movement, we probably shouldn't expect the llm to interpret quaternion rotations,
        # decide if we shouldn't reduce this into more interpretable commands and make static translation utils for all supported ROS message types instead
        self._context += '''
        translation forward and backward:
        "header": {
            "stamp": {
            "sec": 0,
            "nanosec": 0
            },
            "frame_id": "decide based on the given body frame context"
        },
        "pose": {
            "position": {
                "x": "value",
                "y": 0.0,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }
        translation leftward and rightward:
        "header": {
            "stamp": {
            "sec": 0,
            "nanosec": 0
            },
            "frame_id": "decide based on the given body frame context"
        },
        "pose": {
            "position": {
                "x": 0.0,
                "y": "value",
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }
        translation upward and downward:
        "header": {
            "stamp": {
            "sec": 0,
            "nanosec": 0
            },
            "frame_id": "decide based on the given body frame context"
        },
        "pose": {
            "position": {
                "x": 0.0,
                "y": 0.0,
                "z": "value"
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }
        '''
        self._context += 'Make sure to always use this information when the task does not correspond to a specific keyword.\n\n'

        if int(os.getenv("DEBUG", "0")) >= 2: print(f"full prompt context:\n{self._context}")
