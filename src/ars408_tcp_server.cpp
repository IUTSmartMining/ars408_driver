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

namespace ars408
{
    TcpServer::TcpServer(int port)
        : port_(port), server_fd_(-1)
    {}

    TcpServer::~TcpServer()
    {
        if (server_fd_ != -1) {
            close(server_fd_);
        }
    }

    void TcpServer::start()
    {
        struct sockaddr_in address;
        int opt = 1;

        if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd_, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        std::thread(&TcpServer::accept_connections, this).detach();
    }

    void TcpServer::set_data_callback(const std::function<void(const std::string&)>& callback)
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
        char buffer[1024] = { 0 };
        while (true) {
            int valread = read(socket, buffer, 1024);
            if (valread <= 0) {
                break;
            }

            if (data_callback_) {
                data_callback_(std::string(buffer, valread));
            }
        }
        close(socket);
    }
}