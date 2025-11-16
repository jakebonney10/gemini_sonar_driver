#pragma once  // Favor using this over the #ifndef, #define method


// First include your local package stuff
#include "package_defs.hpp"  //  This is where we include all our namespace stuff for the package

// then include external libary stuff
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>



NS_HEAD  // macro for consistantly defining our namespace for the package


/**
 * @brief a minimal publisher class that extends the rclcpp::Node class to
 * demonstrate how to create ROS2 Node
 *
 * This class is used to create a ROS2 publisher that regularly publishes
 * messages to a topic.
 */
class MinimalPublisherNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Minimal Publisher object
     *
     * The constructor initializes the publisher and timer.
     */
    MinimalPublisherNode();

    /**
     * @brief Destructor for the MinimalPublisherNode class.
     */
    ~MinimalPublisherNode();

    /**
     * @brief Structure for holding parameters for the MinimalPublisherNode class.
     */
    struct Parameters
    {
        int timer_period = 500; ///< Timer period in milliseconds
        std::string log_level = "info"; ///< ROS logger level (debug, info, warn, error, fatal)

        struct Topics{
            std::string subscriber_topic = "input_topic";  ///< Name of the input topic
            std::string publisher_topic = "output_topic";  ///< Name of the output topic
        } topics;

        Parameters();
        void declare(MinimalPublisherNode* node);
        void update(MinimalPublisherNode* node);
    };

    /**
     * @brief Structure for holding subscriptions for the MinimalPublisherNode class.
     */
    struct Subscribers
    {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; ///< Shared pointer to the subscription
        void init(MinimalPublisherNode* node);
    };

    /**
     * @brief Structure for holding publishers for the MinimalPublisherNode class.
     */
    struct Publishers
    {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; ///< Shared pointer to the publisher
        void init(MinimalPublisherNode* node);
    };

protected:
    /**
     * @brief Callback function for the timer
     *
     * This function is called periodically by the timer. It publishes
     * a message containing a string with a counter.
     */
    void timer_callback();

    /**
     * @brief Callback function for the subscription
     *
     * This function is called when a message is received on the subscribed topic.
     *
     * @param msg Shared pointer to the received message.
     */
    void subscriptionCallback(std_msgs::msg::String::SharedPtr msg);

    Parameters parameters_;
    Subscribers subscribers_;
    Publishers publishers_;

    std::string message_;
    rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
    size_t count_; ///< Counter for the number of messages published
};

NS_FOOT
