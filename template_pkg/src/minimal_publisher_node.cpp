#include "minimal_publisher_node.hpp"
#include <algorithm>

using namespace std::chrono_literals;

NS_HEAD

MinimalPublisherNode::Parameters::Parameters(){} // default params are defined in header file

void MinimalPublisherNode::Parameters::declare(MinimalPublisherNode* node)
{
    node->declare_parameter("timer_period", timer_period);
    node->declare_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->declare_parameter("topics.publisher_topic", topics.publisher_topic);
    node->declare_parameter("log_level", log_level);
}

void MinimalPublisherNode::Parameters::update(MinimalPublisherNode* node)
{
    node->get_parameter("timer_period", timer_period);
    node->get_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->get_parameter("topics.publisher_topic", topics.publisher_topic);
    node->get_parameter("log_level", log_level);
}

void MinimalPublisherNode::Subscribers::init(MinimalPublisherNode* node)
{
    subscription_ = node->create_subscription<std_msgs::msg::String>(
        node->parameters_.topics.subscriber_topic, 10,
        std::bind(&MinimalPublisherNode::subscriptionCallback, node, std::placeholders::_1));
}

void MinimalPublisherNode::Publishers::init(MinimalPublisherNode* node)
{
    publisher_ = node->create_publisher<std_msgs::msg::String>(node->parameters_.topics.publisher_topic, 10);
}

MinimalPublisherNode::MinimalPublisherNode()
    : Node("minimal_publisher"), count_(0)
{
    parameters_.declare(this);
    parameters_.update(this);
    
    // Set the logger level based on the parameter
    auto logger = this->get_logger();
    std::string log_level_lower = parameters_.log_level;
    std::transform(log_level_lower.begin(), log_level_lower.end(), log_level_lower.begin(), ::tolower);
    
    if (log_level_lower == "debug") {
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    } else if (log_level_lower == "info") {
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
    } else if (log_level_lower == "warn") {
        this->get_logger().set_level(rclcpp::Logger::Level::Warn);
    } else if (log_level_lower == "error") {
        this->get_logger().set_level(rclcpp::Logger::Level::Error);
    } else if (log_level_lower == "fatal") {
        this->get_logger().set_level(rclcpp::Logger::Level::Fatal);
    } else {
        RCLCPP_WARN(logger, "Invalid log level '%s', using default 'info'", parameters_.log_level.c_str());
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
    }
    
    RCLCPP_DEBUG(logger, "Logger level set to: %s", parameters_.log_level.c_str());
    
    subscribers_.init(this);
    publishers_.init(this);

    message_ = "Hello, world! ";

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters_.timer_period), std::bind(&MinimalPublisherNode::timer_callback, this));
}

void MinimalPublisherNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = message_ + std::to_string(count_++);
    RCLCPP_DEBUG(this->get_logger(), "Timer callback triggered, count: %zu", count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publishers_.publisher_->publish(message);
}

void MinimalPublisherNode::subscriptionCallback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Subscription callback triggered");
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    message_ = msg->data;
    // Try running `ros2 topic pub --once /input_topic std_msgs/msg/String "{data: 'Hello from command line'}"` from the command line
}

MinimalPublisherNode::~MinimalPublisherNode() 
{
    RCLCPP_INFO(this->get_logger(), "MinimalPublisherNode destroyed");
}

NS_FOOT
