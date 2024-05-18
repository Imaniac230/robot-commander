from ai_interface import OpenAI, OpenAIParams

import argparse
import time
import roslibpy
import glob
import os


def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument('--key', type=str, default=None, help='OpenAI API key.')
    parser.add_argument('--host', type=str, default='localhost', help='ROS host.')
    parser.add_argument('--port', type=int, default='9090', help='ROS port.')
    parser.add_argument('--topic', type=str, required=True, help='Topic for commanding the robot.')
    parser.add_argument('--type', type=str, required=True, help='Message type for the topic.')

    args = parser.parse_args()
    return args


def main() -> None:
    args = args_factory()

    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()

    context = ["Window={position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}"]
    context += ["Door={position: {x: 10.0, y: -5.0, z: 0.0}, orientation: {w: 0.5}}"]
    context += ["Tomas={position: {x: -12.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}"]
    context += ["Tomas window={position: {x: 11.5, y: -4.2, z: 0.0}, orientation: {w: 0.5}}"]

    messages = ""
    for file in glob.glob("messages/*.txt"):
        messages += "<" + os.path.basename(file).split('.')[0] + ">"
        with open(file, 'r') as rd:
            messages += rd.read()
        messages += "</" + os.path.basename(file).split('.')[0] + ">" + "\n"

    ctx = ""
    for item in context:
        ctx += item + "\n"

    if int(os.getenv("DEBUG", "0")) >= 1:
        if ctx is not None:
            print(f"context:\n{ctx}")
        print(f"messages:\n{messages}")

    with open("prompts/ros-publisher.txt") as f: initial_prompt = f.read()
    initial_prompt += 'Details about the ROS2 messages that you will output:\n'
    initial_prompt += f'The following shows the ROS2 messages in the required JSON format:\n{messages}\n'
    initial_prompt += 'Each message is enclosed inside a tag that defines its ROS2 name (<message-name></message-name>).\n\n'
    if ctx is not None:
        initial_prompt += 'Additional properties of the environment that you are operating in:\n'
        initial_prompt += f'The following lines provide context for mapping any received keywords to corresponding properties of the JSON response:\n{ctx}\n'
        initial_prompt += 'Make sure you always match all JSON property values exactly each time you detect a corresponding keyword.\n\n'

    openai_interface = OpenAI(OpenAIParams(api_key=args.key, initial_chat_prompt=initial_prompt))
    while True:
        try:
            print("Press and hold 'space' to record your command ...")
            messages = openai_interface.get_messages(openai_interface.get_voice_prompt())
            print("Done...")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break
        except:
            continue

    ros_client.terminate()


if __name__ == '__main__':
    main()
