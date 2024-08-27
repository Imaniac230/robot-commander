#ifndef INPUT_GAMEPADNODE_H
#define INPUT_GAMEPADNODE_H

#include <atomic>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <robot_commander_interfaces/msg/state.hpp>

#include <input/DualsenseCtlWrapper.h>
#include <input/GamepadBase.h>

namespace GamepadNodeParameters {
    static constexpr auto deadZone = "dead_zone";
    static constexpr auto maxSpeed = "max_speed";
    static constexpr auto xMultiplier = "x_multiplier";
    static constexpr auto yMultiplier = "y_multiplier";
    static constexpr auto zMultiplier = "z_multiplier";
    static constexpr auto defaultGear = "default_gear";
    static constexpr auto defaultState = "default_state";
}// namespace GamepadNodeParameters

class GamepadNode : public rclcpp::Node, public GamepadBase {
public:
    GamepadNode();
    ~GamepadNode() override;

private:
    void onInitialize();

    void onFastPublishingTimerTick();
    void onLightbarUpdateTimerTick();
    void chatAgentStateCallback(const robot_commander_interfaces::msg::State::SharedPtr state);
    void rosAgentStateCallback(const robot_commander_interfaces::msg::State::SharedPtr state);

    void onButtonPressed(uint8_t button) override;
    void onButtonReleased(uint8_t button) override;
    void onJoystickAxisMoved(uint8_t axis, int16_t value) override;

    void appendClient(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);
    void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);
    inline bool updateLightbarFromState(const robot_commander_interfaces::msg::State state) {
        switch (state.state) {
            case robot_commander_interfaces::msg::State::STATE_UNKNOWN:
                DualsenseCtl(get_logger()).lightbarColor(DualsenseCtl::Color{.brightness = 0});
                return true;
            case robot_commander_interfaces::msg::State::STATE_RECORDING_PROMPT:
                DualsenseCtl(get_logger())
                        .lightbarColor(
                                DualsenseCtl::Color{.blue = 255,
                                                    .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
                lightbarToggle = !lightbarToggle;
                return true;
            case robot_commander_interfaces::msg::State::STATE_TRANSCRIBING:
            case robot_commander_interfaces::msg::State::STATE_RESPONDING:
            case robot_commander_interfaces::msg::State::STATE_SYNTHESISING:
                DualsenseCtl(get_logger())
                        .lightbarColor(
                                DualsenseCtl::Color{.red = 255,
                                                    .green = 255,
                                                    .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
                lightbarToggle = !lightbarToggle;
                return true;
            case robot_commander_interfaces::msg::State::STATE_PLAYING_RESPONSE:
                DualsenseCtl(get_logger())
                        .lightbarColor(
                                DualsenseCtl::Color{.red = 255,
                                                    .green = 255,
                                                    .blue = 255,
                                                    .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
                lightbarToggle = !lightbarToggle;
                return true;
            case robot_commander_interfaces::msg::State::STATE_IDLE:
                DualsenseCtl(get_logger()).lightbarColor(DualsenseCtl::Color{.green = 255, .brightness = 255});
                return false;
            case robot_commander_interfaces::msg::State::STATE_ERROR:
                DualsenseCtl(get_logger()).lightbarColor(DualsenseCtl::Color{.red = 255, .brightness = 255});
                return true;
        }

        return false;
    }

    rclcpp::TimerBase::SharedPtr fastPublishingTimer;
    rclcpp::TimerBase::SharedPtr eventTimer;
    rclcpp::TimerBase::SharedPtr lightbarUpdateTimer;

    rclcpp::Subscription<robot_commander_interfaces::msg::State>::SharedPtr chatAgentStateSubscription;
    rclcpp::Subscription<robot_commander_interfaces::msg::State>::SharedPtr rosAgentStateSubscription;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resetPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enablePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr disablePublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gamepadStatePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recordPromptPublisher;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sitClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr standClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr selfRightClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rolloverClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr powerOnClient;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr powerOffClient;
    //    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clearBehaviorFaultClient;

    std::pair<std::thread, std::atomic_bool> serviceHandlingThread;
    std::array<std::pair<std::optional<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>, std::mutex>, 10>
            clientPool{};

    robot_commander_interfaces::msg::State chatAgentState{};
    robot_commander_interfaces::msg::State rosAgentState{};

    float deadZone{};
    float xMultiplier{};
    float yMultiplier{};
    float zMultiplier{};
    float maxSpeed{};

    bool lightbarToggle = false;

    //TODO(qos): default config, utilize this more in the future
    //  using a foxy-compatible syntax - humble also supports the constructor syntax
    rclcpp::QoS qosSetting = rclcpp::QoS(10);
};

#endif//INPUT_GAMEPADNODE_H
