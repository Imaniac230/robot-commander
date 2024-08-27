#ifndef INPUT_DUALSENSECTLWRAPPER_H
#define INPUT_DUALSENSECTLWRAPPER_H

#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

class DualsenseCtl {
public:
    enum class SpeakerState { Internal, Speaker, Headphone, Both };

    struct Color {
        uint8_t red{};
        uint8_t green{};
        uint8_t blue{};
        uint8_t brightness{};
    };

    DualsenseCtl() = default;
    explicit DualsenseCtl(const rclcpp::Logger &logger) : logger_(logger) {}

    bool speaker(SpeakerState state);

    inline bool volume(const uint8_t value) {
        std::ostringstream oss("dualsensectl volume ", std::ios_base::ate);
        oss << static_cast<int>(value);
        return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
    }

    inline bool lightbarEnable(const bool enabled) {
        std::ostringstream oss("dualsensectl lightbar ", std::ios_base::ate);
        if (enabled) {
            oss << "on";
        } else {
            oss << "off";
        }
        return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
    }

    bool lightbarColor(Color color);

private:
    enum class SystemCommandStatus { Ok, OutputError, SystemError };

    inline SystemCommandStatus runSystemCommand(const std::string &command) {
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

    rclcpp::Logger logger_{rclcpp::get_logger("dualsensectl_wrapper")};
};

inline std::ostream &operator<<(std::ostream &ostream, const DualsenseCtl::Color color) {
    ostream << static_cast<int>(color.red) << " " << static_cast<int>(color.green) << " "
            << static_cast<int>(color.blue) << " " << static_cast<int>(color.brightness);
    return ostream;
}

#endif//INPUT_DUALSENSECTLWRAPPER_H
