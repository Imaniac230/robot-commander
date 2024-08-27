#include <input/DualsenseCtlWrapper.h>

#include <map>

static const std::map<std::string, DualsenseCtl::SpeakerState> validSpeakerStates{
        {"internal", DualsenseCtl::SpeakerState::Internal},
        {"speaker", DualsenseCtl::SpeakerState::Speaker},
        {"headphone", DualsenseCtl::SpeakerState::Headphone},
        {"both", DualsenseCtl::SpeakerState::Both}};

std::ostream &operator<<(std::ostream &stream, const DualsenseCtl::SpeakerState &state) {
    for (const auto &v: validSpeakerStates) {
        if (state == v.second) {
            stream << v.first;
            return stream;
        }
    }

    stream << "invalid";
    return stream;
}

bool DualsenseCtl::speaker(const SpeakerState state) {
    std::ostringstream oss("dualsensectl speaker ", std::ios_base::ate);
    oss << state;
    return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
}

bool DualsenseCtl::lightbarColor(const Color color) {
    std::ostringstream oss("dualsensectl lightbar ", std::ios_base::ate);
    oss << color;
    return runSystemCommand(oss.str()) == SystemCommandStatus::Ok;
}
