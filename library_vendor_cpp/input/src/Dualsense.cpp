#include <input/Dualsense.h>

#include <map>

namespace Dualsense {
    static const std::map<std::string, SpeakerState> validSpeakerStates{{"internal", SpeakerState::Internal},
                                                                        {"speaker", SpeakerState::Speaker},
                                                                        {"headphone", SpeakerState::Headphone},
                                                                        {"both", SpeakerState::Both}};

    std::ostream &operator<<(std::ostream &stream, const SpeakerState &state) {
        for (const auto &v: validSpeakerStates) {
            if (state == v.second) {
                stream << v.first;
                return stream;
            }
        }

        stream << "invalid";
        return stream;
    }

    bool CliWrapper::speaker(const SpeakerState state) {
        std::ostringstream oss("dualsensectl speaker ", std::ios_base::ate);
        oss << state;
        return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
    }

    bool CliWrapper::lightbar(const Color color) {
        std::ostringstream oss("dualsensectl lightbar ", std::ios_base::ate);
        oss << color;
        return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
    }

    bool Device::speaker(SpeakerState state) {
        std::ostringstream oss;
        oss << state;
        return command_speaker(&ds, oss.str().data()) == 0;
    }
}// namespace Dualsense
