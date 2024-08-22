from robot_commander_library.commander import CommanderState
from robot_commander_py import CommanderActionServerInterface, AgentType
from robot_commander_interfaces.action import Respond

import time
import threading as th
from pathlib import Path

import rclpy
from rclpy.action import ActionServer


class ChatCommander(CommanderActionServerInterface):

    def __init__(self):
        super().__init__('chat_commander', agent_type=AgentType.CHAT)

        self.initialize()

        if self.commander is None: raise RuntimeError("Commander was not initialized.")

        self._action_server = ActionServer(self, Respond, 'respond_chat', self.action_callback)

    def action_callback(self, goal_handle):
        self.get_logger().info("Responding ...")

        feedback = Respond.Feedback()
        try:
            feedback.state = CommanderState.UNKNOWN.value
            goal_handle.publish_feedback(feedback)
            t = th.Thread(target=self.commander.respond, kwargs={"audio_file": goal_handle.request.recording_file})
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

            # TODO(tts-server): local tts server is only experimental for now, allow loading the pytorch model with each prompt as well
            if self.pytorch_tts is not None and self.commander.state is CommanderState.IDLE:
                self.get_logger().warn("Loading a non-server pytorch TTS model for response synthesis ...")
                feedback.state = CommanderState.SYNTHESISING.value
                self.get_logger().info(f"Current state: {CommanderState.SYNTHESISING=}")
                goal_handle.publish_feedback(feedback)
                self.pytorch_tts.synthesize(self.commander.last_response, load_model=True)
                feedback.state = CommanderState.IDLE.value
                self.get_logger().info(f"Current state: {CommanderState.IDLE=}")
                goal_handle.publish_feedback(feedback)

        except Exception as e:
            self.get_logger().error(f"Failed to get response from agent, error: '{e}'")
            feedback.state = CommanderState.ERROR.value
            self.get_logger().info(f"Current state: {CommanderState.ERROR=}")
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
            goal_handle.abort()
            result = Respond.Result()
            return result

        output_file: str = self.pytorch_tts.generated_file if self.pytorch_tts is not None else self.commander.tts_generated_file
        self.get_logger().info(f"Done\nsummary:\n\t{goal_handle.request.recording_file} -> "
                               f"({self.commander.api.params.stt_name if self.commander.api.params.stt_name is not None else 'whisper'})\n\t-> '{self.commander.last_transcription}' -> "
                               f"({self.commander.api.params.llm_name if self.commander.api.params.llm_name is not None else 'llama'})\n\t-> '{self.commander.last_response}' -> "
                               f"({self.commander.api.params.tts_name if self.commander.api.params.tts_name is not None else 'bark'})\n\t-> {output_file}")

        goal_handle.succeed()
        result = Respond.Result()
        # TODO(chat-response): the action client will have to find the actual file, so it must currently run on the same machine,
        # think if it would be useful to pass the generated audio stream instead
        # (we will have to verify that the data format is compatible across all possible use cases)
        result.speech_file = str(Path().resolve() / output_file)
        result.text_response = self.commander.last_response
        return result


def main(args=None):
    rclpy.init(args=args)

    commander = ChatCommander()
    rclpy.spin(commander)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
