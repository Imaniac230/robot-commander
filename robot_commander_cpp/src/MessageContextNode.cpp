#include <robot_commander_cpp/nodes/MessageContextNode.hpp>

#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

MessageContextNode::MessageContextNode() : rclcpp::Node("message_context_node") {
    declare_parameter(MessageContextNodeParameters::poseContextFilePath, rclcpp::ParameterValue(""));
    declare_parameter(MessageContextNodeParameters::loadPoseContextFile, rclcpp::ParameterValue(false));

    onInitialize();
}

void MessageContextNode::onInitialize() {
    RCLCPP_INFO(get_logger(), "Starting the message context node.");

    std::string poseContextFile = get_parameter(MessageContextNodeParameters::poseContextFilePath).as_string();
    if (poseContextFile.empty()) {
        std::ostringstream oss("posestamped_", std::ios_base::ate);
        oss << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        poseContextFile = oss.str();
        RCLCPP_WARN(get_logger(), "ROS parameter '%s' was not provided, setting name to '%s'",
                    MessageContextNodeParameters::poseContextFilePath, poseContextFile.c_str());
    }
    contexts.emplace_back(poseContextFile, robot_commander_interfaces::msg::PoseStampedKeywordArray());
    loadFromFile.push_back(get_parameter(MessageContextNodeParameters::loadPoseContextFile).as_bool());

    RCLCPP_INFO(get_logger(), "Initializing subscribers.");
    odometrySubscription = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", qosSetting, std::bind(&MessageContextNode::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Initializing publishers.");
    poseContextPublisher =
            create_publisher<robot_commander_interfaces::msg::PoseStampedKeywordArray>("pose_context", qosSetting);

    RCLCPP_INFO(get_logger(), "Initializing servers.");
    updateContextService = create_service<robot_commander_interfaces::srv::UpdateContext>(
            "update_pose_context",
            std::bind(&MessageContextNode::updateContexts, this, std::placeholders::_1, std::placeholders::_2));
    saveDataFilesService = create_service<std_srvs::srv::Trigger>(
            "save_contexts",
            std::bind(&MessageContextNode::saveContextFiles, this, std::placeholders::_1, std::placeholders::_2));

    if (std::find(loadFromFile.begin(), loadFromFile.end(), true) != loadFromFile.end()) {
        RCLCPP_INFO(get_logger(), "Loading context data from given files.");
        if (loadDataFiles() != LoadStatus::Success) RCLCPP_WARN(get_logger(), "Failed to load some data files.");
    }

    //NOTE: we want to publish the contexts as they are loaded from files, but we cannot do that directly from this constructor,
    // so we define a single-use timer callback that will execute all registered publishing callbacks after the node is initialized
    // and then destroy itself
    initialPublishingTimer =
            create_wall_timer(std::chrono::seconds(5), [this] { executeInitialPublishingCallbacks(); });

    RCLCPP_INFO(get_logger(), "Node initialized.");
}

void MessageContextNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { odometry = *msg; }

void MessageContextNode::updateContexts(
        const robot_commander_interfaces::srv::UpdateContext::Request::SharedPtr request,
        const robot_commander_interfaces::srv::UpdateContext::Response::SharedPtr response) {
    response->success = false;
    response->message = "In progress ...";

    for (std::pair<std::string, ContextType> &c: contexts) {
        if (std::holds_alternative<robot_commander_interfaces::msg::PoseStampedKeywordArray>(c.second)) {
            //TODO(timestamp): be wary of this if the odometry time source changes
            if ((rclcpp::Clock(RCL_ROS_TIME).now() - odometry.header.stamp) <= rclcpp::Duration::from_seconds(1)) {
                response->message = "";

                auto &poses = std::get<robot_commander_interfaces::msg::PoseStampedKeywordArray>(c.second);
                if (auto it = std::find_if(poses.poses.begin(), poses.poses.end(),
                                           [&request](const robot_commander_interfaces::msg::PoseStampedKeyword &p)
                                                   -> bool { return p.keyword == request->keyword; });
                    it != poses.poses.end()) {
                    std::stringstream msg;
                    msg << "Provided keyword: '" << request->keyword
                        << "' is already present, contexts will contain multiple poses with the same keyword. ";
                    RCLCPP_WARN(get_logger(), msg.str());
                    response->message += msg.str();
                }

                robot_commander_interfaces::msg::PoseStampedKeyword newPose;
                newPose.keyword = request->keyword;
                newPose.value.header = odometry.header;
                newPose.value.pose = odometry.pose.pose;
                poses.poses.push_back(newPose);

                poseContextPublisher->publish(poses);

                response->success = true;
                response->message += "Current pose added to context with keyword: '" + newPose.keyword + "'";
            } else {
                const std::string msg("Current pose was not updated for more than 1 second, context was not updated.");
                RCLCPP_WARN(get_logger(), msg);
                response->message = msg;
            }
        }
    }
}

MessageContextNode::LoadStatus MessageContextNode::loadDataFiles() {
    LoadStatus status = LoadStatus::Failed;

    if (contexts.size() != loadFromFile.size()) return status;

    for (std::pair<std::string, ContextType> &c: contexts) {
        if (!c.first.empty() && loadFromFile[(&c - &(*contexts.begin()))]) {
            std::ifstream file(c.first);
            nlohmann::json jsonContext;
            try {
                file >> jsonContext;
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Failed to load context data from file '%s'.", c.first.c_str());
                if (status == LoadStatus::Success) status = LoadStatus::PartialSuccess;
                continue;
            }

            if (std::holds_alternative<robot_commander_interfaces::msg::PoseStampedKeywordArray>(c.second)) {
                auto &poses = std::get<robot_commander_interfaces::msg::PoseStampedKeywordArray>(c.second);
                for (const auto &pose: jsonContext) { poses.poses.emplace_back(pose); }

                initialPublishingCallbacks.emplace_back([this, &poses]() { poseContextPublisher->publish(poses); });

                status = LoadStatus::Success;
            }
        }
    }

    return status;
}

void MessageContextNode::saveContextFiles(const std_srvs::srv::Trigger::Request::SharedPtr,
                                          const std_srvs::srv::Trigger::Response::SharedPtr response) const {
    response->success = false;
    response->message = "";

    for (const std::pair<std::string, ContextType> &c: contexts) {
        if (!c.first.empty()) {
            std::ofstream file;
            file.open(c.first);
            file << c.second;
            file.close();

            response->message += "File '" + c.first + "' saved. ";
            response->success = true;
        }
    }
}

std::ofstream &operator<<(std::ofstream &file, const MessageContextNode::ContextType &context) {
    if (std::holds_alternative<robot_commander_interfaces::msg::PoseStampedKeywordArray>(context)) {
        const auto &poses = get<robot_commander_interfaces::msg::PoseStampedKeywordArray>(context);
        file << "[\n";
        for (auto it = poses.poses.begin(); it != (poses.poses.end() - 1); ++it) {
            nlohmann::json j = *it;
            file << j.dump(2) << ",\n";
        }
        nlohmann::json j = poses.poses.back();
        file << j.dump(2);
        file << "\n]";
    }

    return file;
}

void robot_commander_interfaces::msg::to_json(nlohmann::json &j, const PoseStampedKeyword &message) {
    j = nlohmann::json{
            {"keyword", message.keyword},
            {"value",
             {{"header",
               {{"stamp",
                 {
                         {"sec", message.value.header.stamp.sec},
                         {"nanosec", message.value.header.stamp.nanosec},
                 }},
                {"frame_id", message.value.header.frame_id}}},
              {"pose",
               {
                       {"position",
                        {
                                {"x", message.value.pose.position.x},
                                {"y", message.value.pose.position.y},
                                {"z", message.value.pose.position.z},
                        }},
                       {"orientation",
                        {
                                {"x", message.value.pose.orientation.x},
                                {"y", message.value.pose.orientation.y},
                                {"z", message.value.pose.orientation.z},
                                {"w", message.value.pose.orientation.w},
                        }},
               }}}},
    };
}

void robot_commander_interfaces::msg::from_json(const nlohmann::json &j, PoseStampedKeyword &message) {
    j.at("keyword").get_to(message.keyword);
    j.at("value").get_to(message.value);
}

void geometry_msgs::msg::from_json(const nlohmann::json &j, PoseStamped &message) {
    j.at("header").get_to(message.header);
    j.at("pose").get_to(message.pose);
}

void geometry_msgs::msg::from_json(const nlohmann::json &j, Pose &message) {
    j.at("position").get_to(message.position);
    j.at("orientation").get_to(message.orientation);
}

void geometry_msgs::msg::from_json(const nlohmann::json &j, Point &message) {
    j.at("x").get_to(message.x);
    j.at("y").get_to(message.y);
    j.at("z").get_to(message.z);
}

void geometry_msgs::msg::from_json(const nlohmann::json &j, Quaternion &message) {
    j.at("x").get_to(message.x);
    j.at("y").get_to(message.y);
    j.at("z").get_to(message.z);
    j.at("w").get_to(message.w);
}

void std_msgs::msg::from_json(const nlohmann::json &j, Header &message) {
    j.at("frame_id").get_to(message.frame_id);
    j.at("stamp").get_to(message.stamp);
}

void builtin_interfaces::msg::from_json(const nlohmann::json &j, Time &message) {
    j.at("sec").get_to(message.sec);
    j.at("nanosec").get_to(message.nanosec);
}
