// Copyright 2021 Perception Engine, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ars408_ros/ars408_tcp_node.hpp"

namespace ars408
{
    TcpServerNode::TcpServerNode()
        : Node("ars_tcp_server_node"), tcp_server_()
    {
        ip_ = this->declare_parameter<std::string>("ip_address", "192.168.100.9");
        port_ = this->declare_parameter<int>("port", 50000);

        std::cout << "tcp node started with address: " << ip_ << ":" << port_ << std::endl;

        tcp_server_.set_address(ip_, port_);
        publisher_ = this->create_publisher<can_msgs::msg::Frame>("~/input/frame", 10);
        tcp_server_.set_data_callback([this](const can_msgs::msg::Frame& data) {
            this->publish_data(data);
            });
        tcp_server_.start();
    }

    void TcpServerNode::publish_data(const can_msgs::msg::Frame& can_msg)
    {
        publisher_->publish(can_msg);
    }
}