#include <rclcpp/executors.hpp>

#include <input/GamepadNode.h>

using namespace std::chrono_literals;

GamepadNode::GamepadNode() : rclcpp::Node("gamepad_node"), GamepadBase(get_logger()) {
    declare_parameter(GamepadNodeParameters::deadZone, rclcpp::ParameterValue(0.2));
    declare_parameter(GamepadNodeParameters::maxSpeed, rclcpp::ParameterValue(1.0));
    declare_parameter(GamepadNodeParameters::xMultiplier, rclcpp::ParameterValue(-1.0));
    declare_parameter(GamepadNodeParameters::yMultiplier, rclcpp::ParameterValue(-1.0));
    declare_parameter(GamepadNodeParameters::zMultiplier, rclcpp::ParameterValue(-1.0));
    declare_parameter(GamepadNodeParameters::defaultGear, rclcpp::ParameterValue(1));
    declare_parameter(GamepadNodeParameters::defaultState, rclcpp::ParameterValue("RawMode"));

    onInitialize();
}

GamepadNode::~GamepadNode() {
    RCLCPP_INFO(get_logger(), "Destroying the gamepad node.");
    serviceHandlingThread.second = false;
    if (serviceHandlingThread.first.joinable()) serviceHandlingThread.first.join();
    DualsenseCtl(get_logger()).lightbarEnable(false);
}

void GamepadNode::onInitialize() {
    RCLCPP_INFO(get_logger(), "Starting the gamepad node.");

    const std::lock_guard<std::mutex> lock(controlData.second);
    deadZone = static_cast<float>(get_parameter(GamepadNodeParameters::deadZone).as_double());
    maxSpeed = static_cast<float>(get_parameter(GamepadNodeParameters::maxSpeed).as_double());
    xMultiplier = static_cast<float>(get_parameter(GamepadNodeParameters::xMultiplier).as_double());
    yMultiplier = static_cast<float>(get_parameter(GamepadNodeParameters::yMultiplier).as_double());
    zMultiplier = static_cast<float>(get_parameter(GamepadNodeParameters::zMultiplier).as_double());
    controlData.first.gear = static_cast<int>(get_parameter(GamepadNodeParameters::defaultGear).as_int());
    try {
        defaultGamepadState = GamepadState(get_parameter(GamepadNodeParameters::defaultState).as_string());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Invalid ROS parameter: %s !", GamepadNodeParameters::defaultState);
        throw;
    }
    connectionState.gamepadState = GamepadState::Value::Disconnected;

    RCLCPP_INFO(get_logger(), "Initializing subscribers.");
    chatAgentStateSubscription = create_subscription<robot_commander_interfaces::msg::State>(
            "chat_agent_state", qosSetting,
            std::bind(&GamepadNode::chatAgentStateCallback, this, std::placeholders::_1));
    rosAgentStateSubscription = create_subscription<robot_commander_interfaces::msg::State>(
            "ros_agent_state", qosSetting, std::bind(&GamepadNode::rosAgentStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Initializing publishers.");
    twistPublisher = create_publisher<geometry_msgs::msg::Twist>("output_twist", qosSetting);
    gamepadStatePublisher = create_publisher<std_msgs::msg::String>("gamepad_state", qosSetting);
    resetPublisher = create_publisher<std_msgs::msg::Bool>("reset", qosSetting);
    enablePublisher = create_publisher<std_msgs::msg::Bool>("enable", qosSetting);
    disablePublisher = create_publisher<std_msgs::msg::Bool>("disable", qosSetting);
    recordPromptPublisher = create_publisher<std_msgs::msg::Bool>("record_prompt", qosSetting);
    fastPublishingTimer = create_wall_timer(100ms, [this] { onFastPublishingTimerTick(); });
    eventTimer = create_wall_timer(100ms, [this] { onEventTimerTick(); });
    lightbarUpdateTimer = create_wall_timer(200ms, [this] { onLightbarUpdateTimerTick(); });

    RCLCPP_INFO(get_logger(), "Initializing clients.");
    auto service_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sitClient = create_client<std_srvs::srv::Trigger>("sit", rmw_qos_profile_services_default, service_callback_group);
    standClient =
            create_client<std_srvs::srv::Trigger>("stand", rmw_qos_profile_services_default, service_callback_group);
    selfRightClient = create_client<std_srvs::srv::Trigger>("self_right", rmw_qos_profile_services_default,
                                                            service_callback_group);
    rolloverClient =
            create_client<std_srvs::srv::Trigger>("rollover", rmw_qos_profile_services_default, service_callback_group);
    powerOnClient =
            create_client<std_srvs::srv::Trigger>("power_on", rmw_qos_profile_services_default, service_callback_group);
    powerOffClient = create_client<std_srvs::srv::Trigger>("power_off", rmw_qos_profile_services_default,
                                                           service_callback_group);
    //    clearBehaviorFaultClient = create_client<std_srvs::srv::Trigger>("clear_behavior_fault", rmw_qos_profile_services_default, service_callback_group);
    serviceHandlingThread = std::make_pair(std::thread([this] {
                                               while ((errno != EINTR) && serviceHandlingThread.second.load()) {
                                                   for (auto &client: clientPool) {
                                                       if (client.first.has_value()) {
                                                           callService(client.first.value());
                                                           const std::lock_guard<std::mutex> clientLock(client.second);
                                                           //                    client.first = std::nullopt;
                                                           client.first.reset();
                                                       }
                                                   }
                                               }
                                           }),
                                           true);

    RCLCPP_INFO(get_logger(), "Initializing the dualsense gamepad.");
    DualsenseCtl(get_logger()).speaker(DualsenseCtl::SpeakerState::Both);
    DualsenseCtl(get_logger()).volume(230);
    DualsenseCtl(get_logger()).lightbarEnable(false);
}

void GamepadNode::appendClient(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    for (auto &c: clientPool) {
        if (!c.first.has_value()) {
            const std::lock_guard<std::mutex> lock(c.second);
            c.first = client;
            return;
        }
    }

    RCLCPP_WARN(get_logger(), "Failed to push client '%s', client pool is full.", client->get_service_name());
}

void GamepadNode::callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    RCLCPP_INFO(get_logger(), "Waiting 2 seconds for service '%s' to become available ...", client->get_service_name());
    if (!client->wait_for_service(2s)) {
        if (rclcpp::ok()) {
            RCLCPP_WARN(get_logger(), "Service '%s' not available, cancelling call.", client->get_service_name());
        } else {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Cancelling call.");
        }
        return;
    }

    auto inner_client_callback = [this, &client](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture inner_future) {
        const auto &result = inner_future.get();
        if (result->success) {
            RCLCPP_INFO(get_logger(), "Call to '%s' completed successfully, message: %s.", client->get_service_name(),
                        result->message.data());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call service '%s', message: %s", client->get_service_name(),
                         result->message.data());
        }
    };
    client->async_send_request(request, inner_client_callback);
}

void GamepadNode::onFastPublishingTimerTick() {
    geometry_msgs::msg::Twist twistMsg;
    if (connectionState.gamepadState == GamepadState::Value::RawMode) {
        const float gain = maxSpeed / static_cast<float>(MAX_GEAR) * static_cast<float>(controlData.first.gear);
        //NOTE: axes are switched -> X-axis in robot frame maps to Y-axis on joystick
        twistMsg.linear.x = gain * xMultiplier * controlData.first.axisValues[JoystickAxis::RIGHT_Y];
        twistMsg.linear.y = gain * yMultiplier * controlData.first.axisValues[JoystickAxis::RIGHT_X];
        twistMsg.angular.z = gain * zMultiplier * controlData.first.axisValues[JoystickAxis::LEFT_X];
    } else {
        twistMsg.linear.x = 0.0f;
        twistMsg.linear.y = 0.0f;
        twistMsg.angular.z = 0.0f;
    }
    twistPublisher->publish(twistMsg);

    if (connectionState.gamepadState == GamepadState::Value::Error)
        RCLCPP_ERROR(get_logger(), "Gamepad is in error state !");

    std_msgs::msg::String stateStrMsg;
    stateStrMsg.data = connectionState.gamepadState.asStr();
    gamepadStatePublisher->publish(stateStrMsg);
}

void GamepadNode::onLightbarUpdateTimerTick() {
    if (updateLightbarFromState(chatAgentState)) return;
    updateLightbarFromState(rosAgentState);
}

void GamepadNode::chatAgentStateCallback(const robot_commander_interfaces::msg::State::SharedPtr state) {
    chatAgentState = *state;
}

void GamepadNode::rosAgentStateCallback(const robot_commander_interfaces::msg::State::SharedPtr state) {
    rosAgentState = *state;
}

void GamepadNode::onButtonPressed(uint8_t button) {
    const std::lock_guard<std::mutex> lock(controlData.second);
    std_msgs::msg::Bool msg;
    switch (button) {
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_LEFTSTICK:
            controlData.first.gear = std::max(0, controlData.first.gear - 1);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_RIGHTSTICK:
            controlData.first.gear = std::min(controlData.first.gear + 1, MAX_GEAR);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_X:
            reset();
            msg.data = true;
            resetPublisher->publish(msg);
            //            call_service(clearBehaviorFaultClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_A:
            setMode(GamepadState::Value::AssistedMode);
            msg.data = true;
            enablePublisher->publish(msg);
            appendClient(powerOnClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_B:
            setMode(GamepadState::Value::Disabled);
            if (controlData.first.buttonStates.leftShoulder == ButtonState::Pressed) {
                msg.data = true;
                disablePublisher->publish(msg);
                appendClient(powerOffClient);
            }
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_Y:
            setMode(GamepadState::Value::RawMode);
            msg.data = true;
            enablePublisher->publish(msg);
            appendClient(powerOnClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_GUIDE:
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
            controlData.first.buttonStates.leftShoulder = ButtonState::Pressed;
            //            if (controlData.first.buttonStates.rightShoulder == ButtonState::Pressed) switchMode();
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
            controlData.first.buttonStates.rightShoulder = ButtonState::Pressed;
            //            if (controlData.first.buttonStates.leftShoulder == ButtonState::Pressed) switchMode();
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_START:
            msg.data = true;
            recordPromptPublisher->publish(msg);
            break;
        //testing spot services
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_DPAD_UP:
            appendClient(standClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            appendClient(sitClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_DPAD_LEFT:
            if (controlData.first.buttonStates.leftShoulder == ButtonState::Pressed) appendClient(selfRightClient);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            if (controlData.first.buttonStates.leftShoulder == ButtonState::Pressed) appendClient(rolloverClient);
            break;
        default:
            break;
    }
}

void GamepadNode::onButtonReleased(uint8_t button) {
    const std::lock_guard<std::mutex> lock(controlData.second);
    std_msgs::msg::Bool msg;
    switch (button) {
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_X:
            msg.data = false;
            resetPublisher->publish(msg);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_A:
            msg.data = false;
            enablePublisher->publish(msg);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_B:
            msg.data = false;
            disablePublisher->publish(msg);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_Y:
            msg.data = false;
            enablePublisher->publish(msg);
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
            controlData.first.buttonStates.leftShoulder = ButtonState::Released;
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
            controlData.first.buttonStates.rightShoulder = ButtonState::Released;
            break;
        case SDL_GameControllerButton::SDL_CONTROLLER_BUTTON_START:
            msg.data = false;
            recordPromptPublisher->publish(msg);
            break;
        default:
            break;
    }
}

void GamepadNode::onJoystickAxisMoved(uint8_t axis, int16_t value) {
    const std::lock_guard<std::mutex> lock(controlData.second);
    switch (axis) {
        case SDL_CONTROLLER_AXIS_LEFTX:
            controlData.first.axisValues[JoystickAxis::LEFT_X] = (static_cast<float>(value) / INT16_MAX);
            if (std::abs(controlData.first.axisValues[JoystickAxis::LEFT_X]) < deadZone) {
                controlData.first.axisValues[JoystickAxis::LEFT_X] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_LEFTY:
            controlData.first.axisValues[JoystickAxis::LEFT_Y] = (static_cast<float>(value) / INT16_MAX);
            if (std::abs(controlData.first.axisValues[JoystickAxis::LEFT_Y]) < deadZone) {
                controlData.first.axisValues[JoystickAxis::LEFT_Y] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_RIGHTX:
            controlData.first.axisValues[JoystickAxis::RIGHT_X] = (static_cast<float>(value) / INT16_MAX);
            if (std::abs(controlData.first.axisValues[JoystickAxis::RIGHT_X]) < deadZone) {
                controlData.first.axisValues[JoystickAxis::RIGHT_X] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_RIGHTY:
            controlData.first.axisValues[JoystickAxis::RIGHT_Y] = (static_cast<float>(value) / INT16_MAX);
            if (std::abs(controlData.first.axisValues[JoystickAxis::RIGHT_Y]) < deadZone) {
                controlData.first.axisValues[JoystickAxis::RIGHT_Y] = 0;
            }
            break;
        default:
            break;
    }
}