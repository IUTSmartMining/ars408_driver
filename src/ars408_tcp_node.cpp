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
        : Node("ars_tcp_server_node"), tcp_server_(12345)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("tcp_data", 10);
        tcp_server_.set_data_callback([this](const std::string& data) {
            this->publish_data(data);
            });
        tcp_server_.start();
    }

    void TcpServerNode::publish_data(const std::string& data)
    {
        auto message = std_msgs::msg::String();
        message.data = data;
        publisher_->publish(message);
    }
}