#pragma once
#include "resp_publishers_defs.hpp"
#include "blocks/block.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>


BLOCKS_NS_HEAD

template <typename msg_T, typename block_T>
class RespPub{
public:
  RespPub(rclcpp::Node::SharedPtr node_ptr){
    node_ptr_=node_ptr;
  };

protected:
  rclcpp::Node::SharedPtr node_ptr_;
};

BLOCKS_NS_FOOT
