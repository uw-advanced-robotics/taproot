/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tcp_test_client.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdexcept>
namespace test
{
namespace communication
{
/**
 * This is a test TCP client so that we can test our TCPServer implementation.
 */

TCPClient::TCPClient(const char *hostname, int portno) : portno(portno)
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("TCPClient: ERROR opening socket");
        throw std::runtime_error("SocketError");
    }

    server = gethostbyname(hostname);

    if (server == NULL)
    {
        fprintf(stderr, "TCPClient: ERROR, no such host\n");
        throw std::runtime_error("HostLookupFailure");
    }

    memset((char *)&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("TCPClient: Failed to connect");
        throw std::runtime_error("ConnectError");
    }

    // At this point client is successfully connected, and we can use socket
    // file descriptor to read whatever we want.
}

void TCPClient::Read(char *buffer, int length)
{
    int n = read(sockfd, buffer, length);
    if (n < 0)
    {
        perror("TCPClient: Read error");
        throw std::runtime_error("ReadError");
    }

    // This should basically never happen, but if it does I want to know.
    if (n < length)
    {
        printf("TCPClient: short read occurred");
    }
}

void TCPClient::Write(char *message)
{
    int length = strlen(message);
    int n = write(sockfd, message, length);

    if (n < 0)
    {
        perror("TCPClient: write error");
        throw std::runtime_error("WriteError");
    }
}

}  // namespace communication

}  // namespace test
