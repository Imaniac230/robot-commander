from robot_commander_library.utils import Recorder
from robot_commander_interfaces.action import Respond

from pathlib import Path
from typing import NamedTuple, Optional
import sounddevice as sd
import soundfile as sf
import threading as th
import atomics
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus


class ActionResult(NamedTuple):
    status: GoalStatus = GoalStatus.STATUS_UNKNOWN
    result: Respond.Result = Respond.Result()


class CommanderActionClient(Node):

    def __init__(self):
        super().__init__('commander')

        self._chat_action_client = ActionClient(self, Respond, 'respond_chat')
        self._goal_action_client = ActionClient(self, Respond, 'respond_goal')

        self._recording_subscription = self.create_subscription(Bool, 'record_prompt', self.recording_callback, 10)

        self._processing_thread = th.Thread(target=self._processing_loop)

        self._chat_goal_handle: Optional[ClientGoalHandle] = None
        self._goal_goal_handle: Optional[ClientGoalHandle] = None

        self.is_recording: atomics.UINT = atomics.atomic(width=1, atype=atomics.UINT)
        self.recording_file = str(Path().resolve() / "input_recording.wav")

        # TODO(action-result): could the future.result() be used here directly?
        self.chat_result = ActionResult()
        self.goal_result = ActionResult()

        self.is_recording.store(int(False))
        self._processing_thread.start()

    def __del__(self):
        if (self._chat_goal_handle is not None and
                (self._chat_goal_handle.status == GoalStatus.STATUS_ACCEPTED or self._chat_goal_handle.status == GoalStatus.STATUS_EXECUTING)):
            chat_cancel_future = self._chat_goal_handle.cancel_goal_async()
            chat_cancel_future.add_done_callback()
        if (self._goal_goal_handle is not None and
                (self._goal_goal_handle.status == GoalStatus.STATUS_ACCEPTED or self._goal_goal_handle.status == GoalStatus.STATUS_EXECUTING)):
            goal_cancel_future = self._goal_goal_handle.cancel_goal_async()
            goal_cancel_future.add_done_callback()

        self.is_recording.store(int(False))
        if self._processing_thread.is_alive(): self._processing_thread.join()

    def _processing_loop(self) -> None:
        self.get_logger().info("Commander action client node started.")
        while True:
            try:
                if self.record_new_prompt(): self.send_action_goals(self.recording_file)
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Failed to acquire new prompt, error: '{e}'")
                continue

    async def _record_prompt(self):
        # TODO(chat-prompt): the action server will have to find the actual file, so it must currently run on the same machine,
        # think if it would be useful to pass the generated audio stream instead
        # (we will have to verify that the data format is compatible across all possible use cases)
        async with Recorder(frequency_hz=16000) as r:
            async def stop_recording():
                while self.is_recording.load(): await asyncio.sleep(0.005)
                await r.stop_recording()
                r.save_recording(self.recording_file)

            await asyncio.gather(r.start_recording(), stop_recording())

        # this is only available from python 3.11
        # async with asyncio.TaskGroup() as tg:
        #     async with Recorder(frequency_hz=16000) as r:
        #         async def stop_recording():
        #             while self.is_recording.load(): await asyncio.sleep(0.005)
        #             await r.stop_recording()
        #             r.save_recording(self.recording_file)
        #
        #         tg.create_task(r.start_recording())
        #         tg.create_task(stop_recording())

    def record_new_prompt(self) -> bool:
        if not self.is_recording.load(): return False

        self.get_logger().info(f"Recording prompt to file: {self.recording_file} ...")
        asyncio.run(self._record_prompt())
        self.get_logger().info("Recording finished.")
        return True

    def send_action_goals(self, file: str):
        goal_msg = Respond.Goal()
        goal_msg.recording_file = file

        self.get_logger().info(f"Sending prompt recording '{file}' to chat agent ...")
        self._chat_action_client.wait_for_server()
        chat_action_goal_future = self._chat_action_client.send_goal_async(goal_msg)
        chat_action_goal_future.add_done_callback(self.chat_response_callback)
        self.chat_result = ActionResult(GoalStatus.STATUS_EXECUTING)

        self.get_logger().info(f"Waiting for chat response  ...")
        while self.chat_result.status == GoalStatus.STATUS_EXECUTING: pass
        if self.chat_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Playing back chat response ...")
            data, rate = sf.read(self.chat_result.result.speech_file)
            sd.wait()
            sd.play(data, rate, blocking=False)

        self.get_logger().info(f"Sending prompt recording '{file}' to goal agent ...")
        self._goal_action_client.wait_for_server()
        goal_action_goal_future = self._goal_action_client.send_goal_async(goal_msg)
        goal_action_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_result = ActionResult(GoalStatus.STATUS_EXECUTING)

    def recording_callback(self, msg):
        self.is_recording.store(int(msg.data))

    def chat_response_callback(self, future):
        self._chat_goal_handle = future.result()
        if not self._chat_goal_handle.accepted:
            self.get_logger().warning(f"Chat action server has rejected the goal: {self._chat_goal_handle}.")
            self.chat_result = ActionResult()
            return

        self.get_logger().info("Chat action server goal accepted.")
        get_result_future = self._chat_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.chat_result_callback)

    def goal_response_callback(self, future):
        self._goal_goal_handle = future.result()
        if not self._goal_goal_handle.accepted:
            self.get_logger().warning(f"Goal pose action server has rejected the goal: {self._goal_goal_handle}.")
            self.goal_result = ActionResult()
            return

        self.get_logger().info("Goal pose action server goal accepted.")
        get_result_future = self._goal_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)

    def chat_result_callback(self, future):
        result_handle = future.result()
        self.chat_result = ActionResult(result_handle.status, result_handle.result)

        if self.chat_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Chat action result:\nspeech_file: {self.chat_result.result.speech_file}\ntext_response: {self.chat_result.result.text_response}')
        else:
            self.get_logger().error(f"Chat action goal '{result_handle}' has failed.")

    def goal_result_callback(self, future):
        result_handle = future.result()
        self.goal_result = ActionResult(result_handle.status, result_handle.result)

        if self.goal_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal action result:\nspeech_file: {self.goal_result.result.speech_file}\ntext_response: {self.goal_result.result.text_response}')
        else:
            self.get_logger().error(f"Goal action goal '{result_handle}' has failed.")


def main(args=None):
    rclpy.init(args=args)

    commander = CommanderActionClient()
    rclpy.spin(commander)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
