#ifndef ROBOT_COMMANDER_CPP_MESSAGECONTEXTNODE_H
#define ROBOT_COMMANDER_CPP_MESSAGECONTEXTNODE_H

#include <variant>

#include <rclcpp/node.hpp>

#include <nlohmann/json.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <robot_commander_interfaces/msg/pose_stamped_keyword_array.hpp>
#include <robot_commander_interfaces/srv/update_context.hpp>


namespace MessageContextNodeParameters {
    static constexpr auto poseContextFilePath = "pose_context_file_path";
    static constexpr auto loadDataFiles = "load_data_files";
}// namespace MessageContextNodeParameters

class MessageContextNode : public rclcpp::Node {
public:
    typedef std::variant<robot_commander_interfaces::msg::PoseStampedKeywordArray> ContextType;
    typedef std::list<std::pair<std::string, ContextType>> Contexts;

    friend std::ofstream &operator<<(std::ofstream &file, const ContextType &context);

    MessageContextNode();

private:
    enum class LoadStatus : uint8_t { Success, PartialSuccess, Failed };

    void onInitialize();

    [[nodiscard]] LoadStatus loadDataFiles();

    inline void executeInitialPublishingCallbacks() {
        for (const auto &c: initialPublishingCallbacks) { c(); }
        initialPublishingCallbacks.clear();
        initialPublishingTimer->cancel();
        //NOTE: we're resetting the ptr from the registered callback, this might not be safe
        initialPublishingTimer.reset();
    }
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateContexts(const robot_commander_interfaces::srv::UpdateContext::Request::SharedPtr request,
                        const robot_commander_interfaces::srv::UpdateContext::Response::SharedPtr response);
    void saveContextFiles(const std_srvs::srv::Trigger::Request::SharedPtr,
                          const std_srvs::srv::Trigger::Response::SharedPtr response) const;

    rclcpp::TimerBase::SharedPtr initialPublishingTimer;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscription;
    rclcpp::Service<robot_commander_interfaces::srv::UpdateContext>::SharedPtr updateContextService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveDataFilesService;
    rclcpp::Publisher<robot_commander_interfaces::msg::PoseStampedKeywordArray>::SharedPtr poseContextPublisher;

    nav_msgs::msg::Odometry odometry{};
    Contexts contexts{};

    std::list<std::function<void()>> initialPublishingCallbacks;

    //TODO(qos): default config, utilize this more in the future
    //  using a foxy-compatible syntax - humble also supports the constructor syntax
    rclcpp::QoS qosSetting = rclcpp::QoS(10);
};

//TODO(file-storage): find out if there isn't a more straightforward way of doing all of this,
// use bag files or other standard formats instead?
std::ofstream &operator<<(std::ofstream &file, const MessageContextNode::ContextType &context);
namespace robot_commander_interfaces::msg {
    void to_json(nlohmann::json &j, const PoseStampedKeyword &message);
    void from_json(const nlohmann::json &j, PoseStampedKeyword &message);
}// namespace robot_commander_interfaces::msg
namespace geometry_msgs::msg {
    void from_json(const nlohmann::json &j, PoseStamped &message);
    void from_json(const nlohmann::json &j, Pose &message);
    void from_json(const nlohmann::json &j, Point &message);
    void from_json(const nlohmann::json &j, Quaternion &message);
}// namespace geometry_msgs::msg
namespace std_msgs::msg {
    void from_json(const nlohmann::json &j, Header &message);
}// namespace std_msgs::msg
namespace builtin_interfaces::msg {
    void from_json(const nlohmann::json &j, Time &message);
}// namespace builtin_interfaces::msg

#endif//ROBOT_COMMANDER_CPP_MESSAGECONTEXTNODE_H
