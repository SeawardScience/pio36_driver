#include "pio36_node.hpp"

using namespace std::chrono_literals;

NS_HEAD

MinimalPublisherNode::MinimalPublisherNode()
    : Node("pio36")
{
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisherNode::timer_callback, this));
}

void MinimalPublisherNode::timer_callback()
{

}

NS_FOOT
