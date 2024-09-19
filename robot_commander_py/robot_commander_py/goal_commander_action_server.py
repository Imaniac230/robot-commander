from robot_commander_library.commander import CommanderState
from robot_commander_library.utils import ROSPublisherContext
from robot_commander_py import CommanderActionServerInterface, AgentType
from robot_commander_interfaces.action import Respond

from typing import Optional
import threading as th
import json
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GoalCommander(CommanderActionServerInterface):

    def __init__(self):
        super().__init__('goal_commander', agent_type=AgentType.ROS)

        self.initialize()

        if self.commander is None: raise RuntimeError("Commander was not initialized.")

        if self.commander.api.params.tts_host is not None:
            raise RuntimeError("Commander was initialized with a text-to-speech host, but it is not supported by this node.")
        if self.pytorch_tts is not None: self.get_logger().warning("A text-to-speech pytorch model was initialized, but it will not be used by this node.")
        self.get_logger().info("Goal commander action server node initialized.")

        self._odometry_subscription = self.create_subscription(Odometry, 'odometry', self.odometry_callback, 10)
        self._goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self._action_server = ActionServer(self, Respond, 'respond_goal', self.action_callback)

        self.current_pose: Optional[PoseStamped] = None
        self.last_received_pose = self.get_clock().now()

    def parse_feedback(self) -> Optional[str]:
        if (self.get_clock().now() - self.last_received_pose) > Duration(seconds=1):
            self.get_logger().warning('Goal commander pose feedback was not received for more than 1 second, will not use pose as context.')
            self.current_pose = None

        if self.current_pose is not None: return ROSPublisherContext(str(json.dumps(message_to_ordereddict(self.current_pose))), self.ros_messages_dir).context()
        return None

    def odometry_callback(self, msg: Odometry):
        # TODO(strict-time): decide if we should validate 'now - msg.header.stamp'
        self.last_received_pose = self.get_clock().now()

        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = msg.header.frame_id
        # self.current_pose.pose = msg.pose.pose
        # TODO(float-rounding): isn't there a cleaner way for doing this?
        self.current_pose.pose.position.x = round(msg.pose.pose.position.x, 3)
        self.current_pose.pose.position.y = round(msg.pose.pose.position.y, 3)
        self.current_pose.pose.position.z = round(msg.pose.pose.position.z, 3)
        self.current_pose.pose.orientation.x = round(msg.pose.pose.orientation.x, 3)
        self.current_pose.pose.orientation.y = round(msg.pose.pose.orientation.y, 3)
        self.current_pose.pose.orientation.z = round(msg.pose.pose.orientation.z, 3)
        self.current_pose.pose.orientation.w = round(msg.pose.pose.orientation.w, 3)

    def action_callback(self, goal_handle):
        self.get_logger().info("Generating commands ...")

        feedback = Respond.Feedback()
        try:
            feedback.state = CommanderState.UNKNOWN.value
            goal_handle.publish_feedback(feedback)
            t = th.Thread(target=self.commander.respond, kwargs={"audio_file": goal_handle.request.recording_file, "prompt_context": self.parse_feedback()})
            t.start()
            # TODO(commander-state): we should make the commander state access threadsafe or think about doing this asynchronously
            while t.is_alive():
                if feedback.state != self.commander.state.value:
                    self.get_logger().info(f"Current state: {self.commander.state=}")
                    feedback.state = self.commander.state.value
                    goal_handle.publish_feedback(feedback)
            t.join()
            if feedback.state != self.commander.state.value:
                self.get_logger().info(f"Current state: {self.commander.state=}")
                feedback.state = self.commander.state.value
                goal_handle.publish_feedback(feedback)
            if feedback.state == CommanderState.ERROR.value:
                time.sleep(0.1)
                goal_handle.abort()
                result = Respond.Result()
                return result

            messages = json.loads(self.commander.last_response)
        except Exception as e:
            self.get_logger().error(f"Failed to get response from agent, error: '{e}'")
            feedback.state = CommanderState.ERROR.value
            self.get_logger().info(f"Current state: {CommanderState.ERROR=}")
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
            goal_handle.abort()
            result = Respond.Result()
            return result

        self.get_logger().info(f"Done\nsummary:\n\t{goal_handle.request.recording_file} -> "
                               f"({self.commander.api.params.stt_name if self.commander.api.params.stt_name is not None else 'whisper'})\n\t-> '{self.commander.last_transcription}' -> "
                               f"({self.commander.api.params.llm_name if self.commander.api.params.llm_name is not None else 'llama'})\n\t-> '{self.commander.last_response}'")

        for message in messages:
            try:
                msg = PoseStamped()
                set_message_fields(msg, message)
                msg.header.stamp = self.get_clock().now().to_msg()
            except Exception as e:
                self.get_logger().error(f"Failed to parse response from agent, error: '{e}'")
                goal_handle.abort()
                result = Respond.Result()
                result.text_response = self.commander.last_response
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
