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

class MinimalPublisherNode : public rclcpp::Node
{
public:
    MinimalPublisherNode();

protected:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
};

NS_FOOT
