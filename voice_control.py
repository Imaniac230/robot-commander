import argparse
import time

import roslibpy

from ai_interface import OpenAIInterface


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

    context = ['Peter: ["x": 1.0 "y": 1.0 "w": 1.0]']
    context += ['Milos: ["x": 10.0 "y": -5.0 "w": 0.5]']
    context += ['Tomas: ["x": -12.0 "y": 10.0 "w": 1.0]']
    context += ['Milos window: ["x": 11.5 "y": -4.2 "w": 0.5]']
    openai_interface = OpenAIInterface(key=args.key, context=context)

    while True:
        try:
            input("Press enter to record the next command")
            print("Recording...")
            prompt = openai_interface.get_voice_prompt()
            print(f'Got prompt: "{prompt}"')
            print("Breaking down the goal and creating steps...")
            messages = openai_interface.get_messages(prompt=prompt)
            print("Done...")

            publisher = roslibpy.Topic(ros_client, args.topic, args.type)
            if ros_client.is_connected:
                for message in messages:
                    publisher.publish(roslibpy.Message(message))
                    print(f'Publishing message: "{message}"')
                    time.sleep(1)
        except KeyboardInterrupt:
            break

    ros_client.terminate()


if __name__ == '__main__':
    main()
