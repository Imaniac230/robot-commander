import argparse
import time

import roslibpy

from ai_interface import OpenAI


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
    openai_interface = OpenAI(key=args.key, context=context)

    while True:
        try:
            print("Press and hold 'space' to record your command ...")
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
