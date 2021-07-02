/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TCP_TEST_CLIENT_HPP_
#define TCP_TEST_CLIENT_HPP_

#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace test
{
namespace communication
{
class TCPClient
{
public:
    /**
     * Connect to given server hostname and port number.
     * Code is pretty much just copied from here:
     * http://www.linuxhowtos.org/C_C++/socket.htm
     * For learning about socket coding check Beej's networking guide.
     * Don't overthink it, don't worry about efficiency, just check errors,
     * if something breaks, destroy it. Someone else has already handled
     * the rest at a lower level.
     */
    TCPClient(const char* hostname, int portno);

    /**
     * Reads "length" bytes from the socket into the given buffer.
     */
    void Read(char* buffer, int length);

    /**
     * Writes the given message to the server this client is connected to.
     */
    void Write(char* message);

private:
    int sockfd;
    int portno;
    struct sockaddr_in serv_addr;
    struct hostent* server;
};

}  // namespace communication

}  // namespace test

#endif