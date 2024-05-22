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

#include "ars408_ros/ars408_tcp_server.hpp"

#include <arpa/inet.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace ars408
{
    TcpServer::TcpServer()
    {
        server_fd_ = -1;
    }

    TcpServer::~TcpServer()
    {
        if (server_fd_ != -1) {
            close(server_fd_);
        }
    }

    void TcpServer::set_address(std::string ip, int port)
    {
        ip_address_ = ip;
        port_ = port;
    }

    void TcpServer::start()
    {
        struct sockaddr_in address;
        int opt = 1;

        if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            close(server_fd_);
            exit(EXIT_FAILURE);
        }

        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            close(server_fd_);
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_port = htons(port_);

        if (inet_pton(AF_INET, ip_address_.c_str(), &address.sin_addr) <= 0) {
            perror("invalid address / address not supported");
            close(server_fd_);
            exit(EXIT_FAILURE);
        }

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            close(server_fd_);
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd_, 3) < 0) {
            perror("listen");
            close(server_fd_);
            exit(EXIT_FAILURE);
        }

        std::thread(&TcpServer::accept_connections, this).detach();
    }

    // void TcpServer::set_data_callback(const std::function<void(const std::string&)>& callback)
    void TcpServer::set_data_callback(const std::function<void(const can_msgs::msg::Frame&)>& callback)
    {
        data_callback_ = callback;
    }

    void TcpServer::accept_connections()
    {
        struct sockaddr_in address;
        int addrlen = sizeof(address);

        while (true) {
            int new_socket;
            if ((new_socket = accept(server_fd_, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
                perror("accept");
                continue;
            }

            std::thread(&TcpServer::handle_client, this, new_socket).detach();
        }
    }

    void TcpServer::handle_client(int socket)
    {
        __uint8_t buffer[1024] = { 0 };
        while (true) {
            int valread = read(socket, buffer, 1024);
            if (valread <= 0) {
                break;
            }

            if (data_callback_) {
                // data_callback_(readable_buffer(buffer, valread));
                canfd_frame frame;
                can_msgs::msg::Frame can_msg;
                getCanFdFromBytes(frame, buffer);
                getCanMsgFromCanFd(can_msg, frame);
                data_callback_(can_msg);
            }
        }
        close(socket);
    }

    std::string TcpServer::readable_buffer(__uint8_t* buff, size_t len)
    {
        std::string str;
        char byte_[3] = { 0 };
        for (size_t i = 0; i < len; i++)
        {
            sprintf(byte_, "%02X", buff[i]);
            str += byte_;
            str += " ";
        }
        str.pop_back();
        return str;
    }

    void TcpServer::getCanFdFromBytes(canfd_frame& frame, __uint8_t* buff)
    {
        frame.can_id =
            (buff[4]) |
            (buff[3] << 8) |
            (buff[2] << 16) |
            (buff[1] << 24);

        uint8_t info = ((buff[0] & 0xF0) >> 4);
        frame.is_rtr = (info & 0b0100);
        frame.is_extended = (info & 0b1000);

        frame.len = buff[0] & 0x0F;

        for (size_t i = 0; i < frame.len; i++)
        {
            frame.data[i] = buff[i + 5];
        }
        for (size_t i = frame.len; i < 8; i++)
        {
            frame.data[i] = 0;
        }
    }

    void TcpServer::getCanMsgFromCanFd(can_msgs::msg::Frame& can_msg, canfd_frame& frame)
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

        can_msg.header.stamp.sec = seconds.count();
        can_msg.header.stamp.nanosec = nanoseconds.count();

        can_msg.id = frame.can_id;
        can_msg.is_rtr = frame.is_rtr;
        can_msg.is_extended = frame.is_extended;
        can_msg.is_error = 0;
        can_msg.dlc = frame.len;
        for (size_t i = 0; i < 8; i++)
        {
            can_msg.data[i] = frame.data[i];
        }
    }
}