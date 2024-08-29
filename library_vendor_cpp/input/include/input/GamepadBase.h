#ifndef INPUT_GAMEPADBASE_H
#define INPUT_GAMEPADBASE_H

#include <SDL2/SDL.h>
#include <csignal>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>

#include <input/Dualsense.h>

#include <rclcpp/node.hpp>

void sigtermHandler();

namespace GamepadID {
    enum class Vendor : uint16_t { DualSense = DS_VENDOR_ID };
    enum class Product : uint16_t { DualSense = DS_PRODUCT_ID };
}// namespace GamepadID

class GamepadState {
public:
    enum class Value : int8_t { Error = -1, Disconnected = 0, RawMode = 1, AssistedMode = 2, Disabled = 3 };

    GamepadState() = default;
    explicit GamepadState(Value value) : value(value) {}
    GamepadState(GamepadState const &state) = default;
    explicit GamepadState(const std::string &value) : GamepadState(valueFromStr(value)) {}

    inline GamepadState &operator=(const GamepadState &right) = default;
    inline GamepadState &operator=(const GamepadState::Value right) {
        value = right;
        return *this;
    }
    inline bool operator==(const Value right) const { return value == right; }
    inline bool operator!=(const Value right) const { return value != right; }
    inline bool operator==(const int8_t right) const { return static_cast<int8_t>(value) == right; }
    inline bool operator!=(const int8_t right) const { return static_cast<int8_t>(value) != right; }
    friend std::ostream &operator<<(std::ostream &stream, const GamepadState &state);
    [[nodiscard]] std::string asStr() const;

    [[nodiscard]] inline const Value &getValue() const { return value; }

private:
    Value value{};

    static Value valueFromStr(const std::string &state);
};

bool operator==(GamepadState::Value left, const std::string &right);
inline bool operator==(const std::string &left, const GamepadState::Value right) { return right == left; }

class GamepadBase {
public:
    enum class JoystickAxis { LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y };
    enum class ButtonState : bool { Released, Pressed };

    struct Buttons {
        ButtonState leftShoulder = ButtonState::Released;
        ButtonState rightShoulder = ButtonState::Released;
    };
    struct ConnectionState {
        GamepadState gamepadState{};
        std::optional<GamepadState> rememberedGamepadState = std::nullopt;
    };

    explicit GamepadBase(const std::optional<rclcpp::Logger> &logger = std::nullopt);
    virtual ~GamepadBase();

protected:
    static constexpr int MAX_GEAR = 10;

    struct ControlData {
        int gear = 0;
        bool reset = false;
        std::map<JoystickAxis, float> axisValues{};
        Buttons buttonStates{};
    };

    virtual void onConnected();
    virtual void onDisconnected();
    virtual void switchMode();
    virtual void setMode(GamepadState::Value mode);
    inline virtual void reset() {
        connectionState.gamepadState = defaultGamepadState;
        connectionState.rememberedGamepadState = std::nullopt;
    }

    inline virtual void disable() {
        connectionState.rememberedGamepadState = connectionState.gamepadState;
        connectionState.gamepadState = GamepadState::Value::Disabled;
    }
    inline virtual void enable() {
        if (connectionState.gamepadState == GamepadState::Value::Disabled) {
            connectionState.gamepadState = connectionState.rememberedGamepadState.value_or(defaultGamepadState);
            connectionState.rememberedGamepadState = std::nullopt;
        }
    }

    virtual void onButtonPressed(uint8_t button) = 0;
    virtual void onButtonReleased(uint8_t button) = 0;
    virtual void onJoystickAxisMoved(uint8_t axis, int16_t value) = 0;

    void onEventTimerTick();
    void closeHandle();
    [[nodiscard]] inline bool isDevice(const GamepadID::Vendor vid, const GamepadID::Product pid) const {
        return (SDL_GameControllerGetVendor(gamepadHandler) == static_cast<uint16_t>(vid)) &&
               (SDL_GameControllerGetProduct(gamepadHandler) == static_cast<uint16_t>(pid));
    }

    //TODO(mutex): decide if the connectionState should be mutexed as well
    ConnectionState connectionState{};
    std::pair<ControlData, std::mutex> controlData{};
    GamepadState defaultGamepadState = GamepadState(GamepadState::Value::Disconnected);

private:
    SDL_GameController *gamepadHandler = nullptr;

    rclcpp::Logger logger_{rclcpp::get_logger("gamepad_base")};
};

#endif//INPUT_GAMEPADBASE_H
