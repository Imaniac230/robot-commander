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
    if (isDevice(GamepadID::Vendor::DualSense, GamepadID::Product::DualSense))
        Dualsense::Device(get_logger()).lightbar(false);
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
    agentFeedbackTimer = create_wall_timer(200ms, [this] { onAgentFeedbackTimerTick(); });

    RCLCPP_INFO(get_logger(), "Initializing clients.");
    clientHandlingNode = rclcpp::Node::make_shared("client_handling_node");
    sitClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("sit");
    standClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("stand");
    selfRightClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("self_right");
    rolloverClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("rollover");
    powerOnClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("power_on");
    powerOffClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("power_off");
    //    clearBehaviorFaultClient = clientHandlingNode->create_client<std_srvs::srv::Trigger>("clear_behavior_fault");
    serviceHandlingThread = std::make_pair(
            std::thread([this] {
                while ((errno != EINTR) && serviceHandlingThread.second.load()) {
                    for (auto &[client, future, mutex]: clientPool) {
                        const std::lock_guard<std::mutex> lock(mutex);

                        //TODO(future-timeout): also handle cases when the we timeout for too long
                        if (future.has_value()) {
                            if (rclcpp::spin_until_future_complete(clientHandlingNode->get_node_base_interface(),
                                                                   future.value(), std::chrono::milliseconds(5)) !=
                                rclcpp::FutureReturnCode::TIMEOUT) {
                                future.reset();
                                future = std::nullopt;
                                client.reset();
                                client = std::nullopt;
                            }
                        }

                        if (client.has_value() && !future.has_value()) future = callService(client.value());
                    }
                }
            }),
            true);
}

void GamepadNode::appendClient(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr c) {
    for (auto &[client, future, mutex]: clientPool) {
        const std::lock_guard<std::mutex> lock(mutex);
        if (!client.has_value()) {
            client = c;
            return;
        }
    }

    RCLCPP_WARN(get_logger(), "Failed to push client '%s', client pool is full, removing unsuccessful clients.",
                c->get_service_name());
    for (auto &[client, future, mutex]: clientPool) {
        const std::lock_guard<std::mutex> lock(mutex);
        if (!future.has_value()) {
            client.reset();
            client = std::nullopt;
        }
    }
}

GamepadNode::ClientFutureOptional GamepadNode::callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for service '%s' to become available ...",
                         client->get_service_name());
    if (!client->service_is_ready()) return std::nullopt;

    RCLCPP_INFO(get_logger(), "Service '%s' found.", client->get_service_name());
    auto inner_client_callback = [this, client](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture inner_future) {
        const auto &result = inner_future.get();
        if (result->success) {
            RCLCPP_INFO(get_logger(), "Call to '%s' completed successfully, message: %s.", client->get_service_name(),
                        result->message.data());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call service '%s', message: %s", client->get_service_name(),
                         result->message.data());
        }
    };
    return client->async_send_request(request, inner_client_callback);
}

bool GamepadNode::updateLightbarFromState(const robot_commander_interfaces::msg::State state) {
    Dualsense::Device dualsense(get_logger());
    switch (state.state) {
        case robot_commander_interfaces::msg::State::STATE_UNKNOWN:
            dualsense.lightbar(Dualsense::Color{.brightness = 0});
            return false;
        case robot_commander_interfaces::msg::State::STATE_TRANSCRIBING:
            dualsense.lightbar(
                    Dualsense::Color{.blue = 255, .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
            lightbarToggle = !lightbarToggle;
            return true;
        case robot_commander_interfaces::msg::State::STATE_RESPONDING:
            dualsense.lightbar(Dualsense::Color{.green = 130,
                                                .blue = 255,
                                                .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
            lightbarToggle = !lightbarToggle;
            return true;
        case robot_commander_interfaces::msg::State::STATE_SYNTHESISING:
            dualsense.lightbar(Dualsense::Color{.green = 255,
                                                .blue = 130,
                                                .brightness = static_cast<uint8_t>(lightbarToggle ? 255 : 0)});
            lightbarToggle = !lightbarToggle;
            return true;
        case robot_commander_interfaces::msg::State::STATE_IDLE:
            dualsense.lightbar(Dualsense::Color{.green = 255, .brightness = 255});
            return false;
        case robot_commander_interfaces::msg::State::STATE_ERROR:
            dualsense.lightbar(Dualsense::Color{.red = 255, .brightness = 255});
            return true;
    }

    return false;
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

void GamepadNode::onAgentFeedbackTimerTick() {
    if (isDevice(GamepadID::Vendor::DualSense, GamepadID::Product::DualSense) &&
        (connectionState.gamepadState != GamepadState::Value::Disconnected) &&
        (connectionState.gamepadState != GamepadState::Value::Error)) {
        Dualsense::Device dualsense(get_logger());
        //NOTE: both agents will currently use the same prompt recording
        microphoneLEDToggle =
                (chatAgentState.state == robot_commander_interfaces::msg::State::STATE_RECORDING_PROMPT) &&
                !microphoneLEDToggle;
        dualsense.microphoneLED(microphoneLEDToggle);

        if (!updateLightbarFromState(chatAgentState)) updateLightbarFromState(rosAgentState);

        playerLEDs = (chatAgentState.state == robot_commander_interfaces::msg::State::STATE_PLAYING_RESPONSE)
                             ? ++playerLEDs
                             : Dualsense::Players();
        dualsense.playerLEDs(playerLEDs);
    }
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