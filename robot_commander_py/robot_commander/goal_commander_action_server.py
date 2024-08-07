from robot_commander import CommanderActionServerInterface, AgentType
from robot_commander_interfaces.action import Respond

import json
import time

import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped


class GoalCommander(CommanderActionServerInterface):

    def __init__(self):
        super().__init__('goal_commander', agent_type=AgentType.ROS)

        self.initialize()

        if self.commander is None: raise RuntimeError("Commander was not initialized.")

        if self.commander.params.tts_host is not None:
            raise RuntimeError("Commander was initialized with a text-to-speech host, but it is not supported by this node.")
        if self.pytorch_tts is not None: self.get_logger().warning("A text-to-speech pytorch model was initialized, but it will not be used by this node.")

        self._goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self._action_server = ActionServer(self, Respond, 'respond_goal', self.action_callback)

    def action_callback(self, goal_handle):
        self.get_logger().info("Generating commands ...")

        try:
            messages = json.loads(self.commander.respond(goal_handle.request.recording_file, response_format=self.grammar_file, system_prompt=self.system_prompt))
        except Exception as e:
            self.get_logger().error(f"Failed to get response from agent, error: '{e}'")
            goal_handle.abort()
            result = Respond.Result()
            return result

        self.get_logger().info(f"Done\nsummary:\n\t{goal_handle.request.recording_file} -> "
                               f"({self.commander.params.stt_name if self.commander.params.stt_name is not None else 'whisper'})\n\t-> '{self.commander.last_transcription}' -> "
                               f"({self.commander.params.llm_name if self.commander.params.llm_name is not None else 'llama'})\n\t-> '{self.commander.last_response}'")

        for message in messages:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            try:
                msg.header.frame_id = str(message["header"]["frame_id"])
                msg.pose.position.x = float(message["pose"]["position"]["x"])
                msg.pose.position.y = float(message["pose"]["position"]["y"])
                msg.pose.position.z = float(message["pose"]["position"]["z"])
                msg.pose.orientation.x = float(message["pose"]["orientation"]["x"])
                msg.pose.orientation.y = float(message["pose"]["orientation"]["y"])
                msg.pose.orientation.z = float(message["pose"]["orientation"]["z"])
                msg.pose.orientation.w = float(message["pose"]["orientation"]["w"])
            except Exception as e:
                self.get_logger().error(f"Failed to parse response from agent, error: '{e}'")
                goal_handle.abort()
                result = Respond.Result()
                return result

            self.get_logger().info(f"Publishing goal: '{msg}'")
            self._goal_publisher.publish(msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Respond.Result()
        result.text_response = self.commander.last_response
        return result


def main(args=None):
    rclpy.init(args=args)

    commander = GoalCommander()
    rclpy.spin(commander)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
