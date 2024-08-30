#include <iostream>

#include <input/Dualsense.h>
#include <input/GamepadBase.h>

using namespace std::chrono_literals;

static const std::map<std::string, GamepadState::Value> validGamepadStates{
        {"Error", GamepadState::Value::Error},
        {"Disconnected", GamepadState::Value::Disconnected},
        {"RawMode", GamepadState::Value::RawMode},
        {"AssistedMode", GamepadState::Value::AssistedMode},
        {"Disabled", GamepadState::Value::Disabled}};

void sigtermHandler() {
    std::cout << "Caught SIGTERM, closing all SDL instances." << std::endl;
    SDL_Quit();// TODO: This does not seem to be actually necessary, just the
               // printout is enough to bypass the systemd time-outing???
}

GamepadBase::GamepadBase(const std::optional<rclcpp::Logger> &logger) {
    if (logger.has_value()) logger_ = logger.value();

    const std::lock_guard<std::mutex> lock(controlData.second);
    controlData.first.axisValues[JoystickAxis::LEFT_X] = 0;
    controlData.first.axisValues[JoystickAxis::LEFT_Y] = 0;
    controlData.first.axisValues[JoystickAxis::RIGHT_X] = 0;
    controlData.first.axisValues[JoystickAxis::RIGHT_Y] = 0;

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) { throw std::runtime_error("Failed to init SDL2."); }

    signal(SIGTERM, reinterpret_cast<__sighandler_t>(sigtermHandler));
}

GamepadBase::~GamepadBase() {
    RCLCPP_INFO(logger_, "Destroying the GamepadNode node.");
    SDL_Quit();
}

void GamepadBase::closeHandle() {
    SDL_GameControllerClose(gamepadHandler);
    gamepadHandler = nullptr;
}

void GamepadBase::onEventTimerTick() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        //        RCLCPP_WARN(logger_, "registered event: %d", event.type);
        switch (event.type) {
            case SDL_CONTROLLERAXISMOTION:
                onJoystickAxisMoved(event.caxis.axis, event.caxis.value);
                break;
            case SDL_CONTROLLERBUTTONDOWN:
                onButtonPressed(event.cbutton.button);
                break;
            case SDL_CONTROLLERBUTTONUP:
                onButtonReleased(event.cbutton.button);
                break;
            case SDL_CONTROLLERDEVICEADDED:
                onConnected();
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                onDisconnected();
                break;
            default:
                break;
        }
    }
}

void GamepadBase::onConnected() {
    if (gamepadHandler == nullptr) {
        gamepadHandler = SDL_GameControllerOpen(0);
        connectionState.gamepadState = connectionState.rememberedGamepadState.value_or(defaultGamepadState);
        connectionState.rememberedGamepadState = std::nullopt;
        RCLCPP_WARN(logger_, "New gamepad opened.");
        if (isDevice(GamepadID::Vendor::DualSense, GamepadID::Product::DualSense)) {
            RCLCPP_INFO(logger_, "Initializing the dualsense gamepad.");
            Dualsense::Device dualsense(logger_);
            dualsense.speaker(Dualsense::SpeakerState::Both);
            dualsense.volume(230);
            //NOTE: blinking lightbar to signalize connection
            //NOTE: leaving lightbar explicitly on seems to conflict with the color settings
            dualsense.lightbar(true);
            dualsense.lightbar(false);
        }
    } else {
        RCLCPP_ERROR(logger_, "Attempted to open multiple gamepads at once.");
    }
}

void GamepadBase::onDisconnected() {
    closeHandle();

    if (!connectionState.rememberedGamepadState) connectionState.rememberedGamepadState = connectionState.gamepadState;
    connectionState.gamepadState = GamepadState::Value::Disconnected;

    //TODO(edge-states): think more about potential dangerous edge cases like this one
    const std::lock_guard<std::mutex> lock(controlData.second);
    controlData.first.axisValues[JoystickAxis::LEFT_X] = 0;
    controlData.first.axisValues[JoystickAxis::LEFT_Y] = 0;
    controlData.first.axisValues[JoystickAxis::RIGHT_X] = 0;
    controlData.first.axisValues[JoystickAxis::RIGHT_Y] = 0;

    RCLCPP_WARN(logger_, "Gamepad disconnected.");
}

void GamepadBase::switchMode() {
    switch (connectionState.gamepadState.getValue()) {
        case GamepadState::Value::RawMode:
            connectionState.gamepadState = GamepadState::Value::AssistedMode;
            connectionState.rememberedGamepadState = std::nullopt;
            break;
        case GamepadState::Value::AssistedMode:
            connectionState.gamepadState = GamepadState::Value::RawMode;
            connectionState.rememberedGamepadState = std::nullopt;
            break;
        default:
            break;
    }
}

void GamepadBase::setMode(const GamepadState::Value mode) {
    switch (mode) {
        case GamepadState::Value::RawMode:
            connectionState.gamepadState = GamepadState::Value::RawMode;
            connectionState.rememberedGamepadState = std::nullopt;
            break;
        case GamepadState::Value::AssistedMode:
            connectionState.gamepadState = GamepadState::Value::AssistedMode;
            connectionState.rememberedGamepadState = std::nullopt;
            break;
        case GamepadState::Value::Disabled:
            connectionState.gamepadState = GamepadState::Value::Disabled;
            connectionState.rememberedGamepadState = std::nullopt;
            break;
        default:
            break;
    }
}

bool operator==(const GamepadState::Value left, const std::string &right) {
    const auto validState = validGamepadStates.find(right);
    return (validState != validGamepadStates.end()) && (validState->second == left);
}

std::ostream &operator<<(std::ostream &stream, const GamepadState &state) {
    for (const auto &v: validGamepadStates) {
        if (state.getValue() == v.second) {
            stream << v.first;
            return stream;
        }
    }

    stream << "Unknown";
    return stream;
}

std::string GamepadState::asStr() const {
    std::ostringstream oss;
    oss << *this;
    return oss.str();
}

GamepadState::Value GamepadState::valueFromStr(const std::string &state) {
    const auto validState = validGamepadStates.find(state);
    if (validState != validGamepadStates.end()) return validState->second;


    std::ostringstream oss;
    oss << "Invalid state provided: '" << state << "' !" << std::endl;
    throw std::runtime_error{oss.str()};
}