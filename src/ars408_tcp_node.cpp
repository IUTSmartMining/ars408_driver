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

TcpServerNode::TcpServerNode(const rclcpp::NodeOptions& node_options)
    : Node("ars408_tcp_node", node_options), tcp_server_send_(), tcp_server_receive_()
{
    ip_ = this->declare_parameter<std::string>("ip_address", "192.168.100.9");
    port_send_ = this->declare_parameter<int>("port_send", 50000);
    port_receive_ = this->declare_parameter<int>("port_receive", 50001);

    tcp_server_send_.set_address(ip_, port_send_);
    tcp_server_receive_.set_address(ip_, port_receive_);

    publisher_ = this->create_publisher<can_msgs::msg::Frame>("~/input/frame", 10);
    tcp_server_send_.set_data_callback_send([this](const can_msgs::msg::Frame& data) {
        this->publish_data(data);
    });

    subscription_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
        "~/input/gps", 10, std::bind(&TcpServerNode::gps_data_callback, this, std::placeholders::_1));
    tcp_server_receive_.set_data_callback_receive([this](double& velocity, double& yawRate) {
        this->receive_data_callback(velocity, yawRate);
    });

    tcp_server_send_.start();
    RCLCPP_INFO(this->get_logger(), "tcp node sender started with address: %s:%d", ip_.c_str(), port_send_);
    
    tcp_server_receive_.start();
    RCLCPP_INFO(this->get_logger(), "tcp node receiver started with address: %s:%d", ip_.c_str(), port_receive_);
}

void TcpServerNode::publish_data(const can_msgs::msg::Frame& can_msg)
{
    publisher_->publish(can_msg);
}

void TcpServerNode::gps_data_callback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_report_msg)
{
    velocity_report_msg_ = velocity_report_msg;
    first_velocity_report_ = true;
}

void TcpServerNode::receive_data_callback(double& velocity, double& yawRate)
{
    if (!first_velocity_report_) {
        velocity = 0;
        yawRate = 0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "No velocity report received yet. Setting velocity and yawRate to 0.");
        return;
    }

    velocity = velocity_report_msg_->longitudinal_velocity;
    yawRate = velocity_report_msg_->heading_rate;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TcpServerNode)
