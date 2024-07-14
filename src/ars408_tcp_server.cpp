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

void TcpServer::set_data_callback_send(const std::function<void(const can_msgs::msg::Frame&)>& callback)
{
    data_callback_send_ = callback;
}

void TcpServer::set_data_callback_receive(const std::function<void(double& velocity, double& yawRate)>& callback)
{
    data_callback_receive_ = callback;
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

        if (data_callback_send_) {
            can_msgs::msg::Frame can_msg;
            get_can_msg_from_bytes(can_msg, buffer);
            data_callback_send_(can_msg);
        }

        if (data_callback_receive_) {
            __uint8_t buff[13] = { 0 };
            data_callback_receive_(velocity_, yaw_rate_);

            if (turn_) {
                get_bytes_from_gps_velocity(buff, velocity_);
            } else {
                get_bytes_from_gps_yaw_rate(buff, yaw_rate_);
            }
            int valsend = send(socket, buff, 13, 0);
            if (valsend != 13) {
                perror("gps feedback cannot send");
            }
            turn_ = !turn_;
        }
    }
    close(socket);
}

void TcpServer::get_can_msg_from_bytes(can_msgs::msg::Frame& can_msg, __uint8_t* buff)
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    can_msg.header.stamp.sec = seconds.count();
    can_msg.header.stamp.nanosec = nanoseconds.count();

    can_msg.id =
        (buff[4]) |
        (buff[3] << 8) |
        (buff[2] << 16) |
        (buff[1] << 24);

    uint8_t info = ((buff[0] & 0xF0) >> 4);
    can_msg.is_rtr = (info & 0b0100);
    can_msg.is_extended = (info & 0b1000);

    can_msg.dlc = (buff[0] & 0x0F);

    for (int i = 0; i < std::min((int)can_msg.dlc, 8); i++) {
        can_msg.data[i] = buff[i + 5];
    }
    for (int i = std::min((int)can_msg.dlc, 8); i < 8; i++) {
        can_msg.data[i] = 0;
    }
}

void TcpServer::get_bytes_from_gps_velocity(__uint8_t* buff, double velocity)
{
    buff[0] = 0x0F & 2; // data length is 2 byte

    // speed id
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x03;
    buff[4] = 0x00;

    velocity /= 3.6; // convert from km/h to m/s
    int speed = abs(velocity * 50.); // scale with resolution 0.02
    speed = std::min(speed, 8190); // max range velocity

    // 2 MSBs of first data byte is direction
    if (velocity == 0.) buff[5] = 0b0000'0000;
    else if (velocity > 0.) buff[5] = 0b0100'0000;
    else buff[5] = 0b1000'0000;

    // 5 LSBs of first data byte is MSBs of speed
    buff[5] |= ((speed & 0x1F'00) >> 8);
    
    // second data byte is LSBs of speed
    buff[6] = (speed & 0xFF);

    for (size_t i = 7; i < 13; i++)
    {
        buff[i] = 0;
    }
}

void TcpServer::get_bytes_from_gps_yaw_rate(__uint8_t* buff, double yawRate)
{
    buff[0] = 0x0F & 2; // data length is 2 byte

    // yaw rate id
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x03;
    buff[4] = 0x01;

    // convert to range [-180,180]
    if (yawRate >= 180.) yawRate -= 360.;
    else if (yawRate <= -180.) yawRate += 360.;

    yawRate += 327.68; // add offset
    yawRate *= 100; // scale with resolution 0.01
    int yr = yawRate;

    buff[5] = ((yr & 0xFF'00) >> 8);
    buff[6] = (yr & 0xFF);

    for (size_t i = 7; i < 13; i++)
    {
        buff[i] = 0;
    }
}

std::string TcpServer::readable_buffer(__uint8_t* buff, size_t len)
{
    std::string str;
    char byte_[3] = { 0 };
    for (size_t i = 0; i < len; i++) {
        sprintf(byte_, "%02X", buff[i]);
        str += byte_;
        str += " ";
    }
    str.pop_back();
    return str;
}
