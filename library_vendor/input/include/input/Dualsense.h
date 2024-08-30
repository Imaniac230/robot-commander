#ifndef INPUT_DUALSENSE_H
#define INPUT_DUALSENSE_H

#include <sstream>
#include <string>

extern "C" {
#include <dualsensectl/dualsense.h>
}

#include <rclcpp/rclcpp.hpp>

namespace Dualsense {
    enum class SpeakerState { Internal, Speaker, Headphone, Both };

    struct Color {
        uint8_t red{};
        uint8_t green{};
        uint8_t blue{};
        uint8_t brightness{};
    };

    class Players {
    public:
        enum class Value : uint8_t { None = 0, One = 1, Two = 2, Three = 3, Four = 4, Five = 5 };

        Players() : value(Value::None) {}
        explicit Players(const Value v) : value(v) {}
        Players(Players const &other) = default;

        inline Players &operator=(const Players &right) = default;
        inline Players &operator=(const Players::Value right) {
            value = right;
            return *this;
        }
        inline Players operator++() {
            const uint8_t rawValue = (static_cast<uint8_t>(value) + 1) % 6;
            value = static_cast<Value>((rawValue == 0) ? rawValue + 1 : rawValue);
            return *this;
        }

        [[nodiscard]] inline int getValue() const { return static_cast<int>(value); }

    private:
        Value value = Value::None;
    };

    class Interface {
    public:
        Interface() = default;
        explicit Interface(const rclcpp::Logger &logger) : logger_(logger) {}

        [[nodiscard]] virtual bool speaker(SpeakerState state) = 0;
        [[nodiscard]] virtual bool volume(uint8_t value) = 0;
        [[nodiscard]] virtual bool lightbar(bool enabled) = 0;
        [[nodiscard]] virtual bool lightbar(Color color) = 0;
        [[nodiscard]] virtual bool playerLEDs(Players number) = 0;
        [[nodiscard]] virtual bool microphoneLED(bool enabled) = 0;

    protected:
        rclcpp::Logger logger_{rclcpp::get_logger("dualsense")};
    };

    class CliWrapper : private Interface {
    public:
        CliWrapper() = default;
        explicit CliWrapper(const rclcpp::Logger &logger) : Interface(logger) {}

        [[nodiscard]] bool speaker(SpeakerState state) override;

        [[nodiscard]] inline bool volume(const uint8_t value) override {
            std::ostringstream oss("dualsensectl volume ", std::ios_base::ate);
            oss << static_cast<int>(value);
            return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
        }

        [[nodiscard]] inline bool lightbar(const bool enabled) override {
            std::ostringstream oss("dualsensectl lightbar ", std::ios_base::ate);
            if (enabled) {
                oss << "on";
            } else {
                oss << "off";
            }
            return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
        }

        [[nodiscard]] bool lightbar(Color color) override;

        [[nodiscard]] inline bool playerLEDs(const Players number) override {
            std::ostringstream oss("dualsensectl player-leds ", std::ios_base::ate);
            oss << number.getValue();
            return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
        }

        [[nodiscard]] inline bool microphoneLED(const bool enabled) override {
            std::ostringstream oss("dualsensectl microphone-led ", std::ios_base::ate);
            if (enabled) {
                oss << "on";
            } else {
                oss << "off";
            }
            return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
        }

    private:
        enum class SystemCommandStatus { Ok, OutputError, SystemError };

        [[nodiscard]] inline SystemCommandStatus runSystemCommand(const std::string &command) const {
            const int e = std::system(command.c_str());
            if (e != 0) {
                RCLCPP_ERROR(logger_, "Failed to execute the '%s' command (msg: %s, status: %d). ", command.c_str(),
                             ((e < 0)          ? strerror(errno)
                              : (WIFEXITED(e)) ? strerror(WEXITSTATUS(e))
                                               : std::to_string(e))
                                     .c_str(),
                             e);
            }

            //NOTE: 256 is the same as 1 -> Command executed successfully with internal exit code 1
            if ((e == 1) || (e == 256)) return SystemCommandStatus::OutputError;
            //NOTE: Fatal problem in execution or malformed command
            if (e != 0) return SystemCommandStatus::SystemError;
            //NOTE: Command executed successfully with internal exit code 0
            return SystemCommandStatus::Ok;
        }
    };

    class Device : private Interface {
    public:
        Device() : Interface() { initializeDevice(); }
        explicit Device(const rclcpp::Logger &logger) : Interface(logger) { initializeDevice(); }
        virtual ~Device() { dualsense_destroy(&ds); }

        [[nodiscard]] bool speaker(SpeakerState state) override;

        [[nodiscard]] bool volume(const uint8_t value) override { return command_volume(&ds, value) == 0; }

        [[nodiscard]] bool lightbar(const bool enabled) override {
            std::ostringstream oss;
            if (enabled) {
                oss << "on";
            } else {
                oss << "off";
            }
            return command_lightbar1(&ds, oss.str().data()) == 0;
        }

        [[nodiscard]] bool lightbar(Color color) override {
            return command_lightbar3(&ds, color.red, color.green, color.blue, color.brightness) == 0;
        }

        [[nodiscard]] bool playerLEDs(const Players number) override {
            return command_player_leds(&ds, static_cast<uint8_t>(number.getValue())) == 0;
        }

        [[nodiscard]] bool microphoneLED(const bool enabled) override {
            std::ostringstream oss;
            if (enabled) {
                oss << "on";
            } else {
                oss << "off";
            }
            return command_microphone_led(&ds, oss.str().data()) == 0;
        }

    private:
        inline void initializeDevice() {
            if (!dualsense_init(&ds, nullptr)) throw std::runtime_error("Failed to initialize the dualsense gamepad.");
        }

        dualsense ds{};
    };

    inline std::ostream &operator<<(std::ostream &ostream, const Color color) {
        ostream << static_cast<int>(color.red) << " " << static_cast<int>(color.green) << " "
                << static_cast<int>(color.blue) << " " << static_cast<int>(color.brightness);
        return ostream;
    }
}// namespace Dualsense

#endif//INPUT_DUALSENSE_H
