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

#ifndef ARS408_ROS__ARS408_TCP_NODE_HPP_
#define ARS408_ROS__ARS408_TCP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "ars408_ros/ars408_tcp_server.hpp"

namespace ars408
{
    class TcpServerNode : public rclcpp::Node
    {
    public:
        TcpServerNode();

    private:
        void publish_data(const std::string& data);

        std::string ip_;
        int port_;

        TcpServer tcp_server_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    };
}

#endif  // ARS408_ROS__ARS408_TCP_NODE_HPP_