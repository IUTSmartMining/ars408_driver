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

#ifndef ARS408_ROS__ARS408_TCP_SERVER_HPP_
#define ARS408_ROS__ARS408_TCP_SERVER_HPP_

#include "ars408_ros/ars408_commands.hpp"

#include "can_msgs/msg/frame.hpp"
#include <unistd.h>
#include <string>
#include <functional>

namespace ars408
{
    class TcpServer
    {
    public:
        TcpServer();
        ~TcpServer();
        void set_address(std::string ip, int port);
        void start();
        // void set_data_callback(const std::function<void(const std::string&)>& callback);
        void set_data_callback(const std::function<void(const can_msgs::msg::Frame&)>& callback);

    private:
        void accept_connections();
        void handle_client(int socket);
        std::string readable_buffer(__uint8_t* buff, size_t len);
        void getCanFdFromBytes(canfd_frame& frame, __uint8_t* buff);
        void getCanMsgFromCanFd(can_msgs::msg::Frame& can_msg, canfd_frame& frame);

        std::string ip_address_;
        int port_;
        int server_fd_;
        // std::function<void(const std::string&)> data_callback_;
        std::function<void(const can_msgs::msg::Frame&)> data_callback_;
    };
}

#endif  // ARS408_ROS__ARS408_TCP_SERVER_HPP_
