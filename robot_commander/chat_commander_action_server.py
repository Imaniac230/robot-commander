from robot_commander import CommanderActionServerInterface

import rclpy


class ChatCommander(CommanderActionServerInterface):

    def __init__(self):
        super().__init__('chat_commander')

        self.initialize()
        self.launch()

    def launch(self): pass


def main(args=None):
    rclpy.init(args=args)

    commander = ChatCommander()
    rclpy.spin(commander)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
