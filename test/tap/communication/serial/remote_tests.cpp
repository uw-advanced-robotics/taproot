/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

using namespace testing;
using namespace tap;
using namespace tap::arch::clock;
using namespace tap::communication::serial;

class RemoteTest : public Test
{
protected:
    RemoteTest() : drivers(), remote(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.uart, read(_, _)).WillByDefault(Invoke(this, &RemoteTest::handleRead));
    }

    void encodeRemoteData()
    {
        int16_t rhE = rh + 1024;
        int16_t rvE = rv + 1024;
        int16_t lhE = lh + 1024;
        int16_t lvE = lv + 1024;
        int16_t wheelE = wheel + 1024;

        encodedRemoteData.push_back(rhE);
        encodedRemoteData.push_back((rvE << 3) | (0x07 & (rhE >> 8)));
        encodedRemoteData.push_back((lhE << 6) | (0x3f & (rvE >> 5)));
        encodedRemoteData.push_back(lhE >> 2);
        encodedRemoteData.push_back((lvE << 1) | (0x01 & (lhE >> 10)));
        encodedRemoteData.push_back(
            (static_cast<uint8_t>(lss) << 6) | (static_cast<uint8_t>(rss) << 4) |
            (0x0f & (lvE >> 7)));
        encodedRemoteData.push_back(mx);
        encodedRemoteData.push_back(mx >> 8);
        encodedRemoteData.push_back(my);
        encodedRemoteData.push_back(my >> 8);
        encodedRemoteData.push_back(mz);
        encodedRemoteData.push_back(mz >> 8);
        encodedRemoteData.push_back(static_cast<uint8_t>(lb));
        encodedRemoteData.push_back(static_cast<uint8_t>(rb));
        encodedRemoteData.push_back(keys);
        encodedRemoteData.push_back(keys >> 8);
        encodedRemoteData.push_back(wheelE);
        encodedRemoteData.push_back(wheelE >> 8);
    }

    bool handleRead(Uart::UartPort, uint8_t *data)
    {
        if (encodedRemoteData.size() == 0)
        {
            return false;
        }
        *data = encodedRemoteData.front();
        encodedRemoteData.pop_front();
        return true;
    }

    void evaluateRemoteInfo()
    {
        EXPECT_EQ(rh / 660.0f, remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL));
        EXPECT_EQ(rv / 660.0f, remote.getChannel(Remote::Channel::RIGHT_VERTICAL));
        EXPECT_EQ(lh / 660.0f, remote.getChannel(Remote::Channel::LEFT_HORIZONTAL));
        EXPECT_EQ(lv / 660.0f, remote.getChannel(Remote::Channel::LEFT_VERTICAL));
        EXPECT_EQ(mx, remote.getMouseX());
        EXPECT_EQ(my, remote.getMouseY());
        EXPECT_EQ(mz, remote.getMouseZ());
        EXPECT_EQ(wheel, remote.getWheel());
        EXPECT_EQ(lss, remote.getSwitch(Remote::Switch::LEFT_SWITCH));
        EXPECT_EQ(rss, remote.getSwitch(Remote::Switch::RIGHT_SWITCH));
        EXPECT_EQ(lb, remote.getMouseL());
        EXPECT_EQ(rb, remote.getMouseR());

        for (size_t i = 0; i < sizeof(keys) * 8; i++)
        {
            EXPECT_EQ((keys >> i) & 0x1, remote.keyPressed(static_cast<Remote::Key>(i)));
        }
    }

    ClockStub clock;
    Drivers drivers;
    Remote remote;
    std::deque<uint8_t> encodedRemoteData;

    int16_t rh = 0;
    int16_t rv = 0;
    int16_t lh = 0;
    int16_t lv = 0;
    uint16_t keys = 0;
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;
    int16_t wheel = 0;
    Remote::SwitchState lss = Remote::SwitchState::UNKNOWN;
    Remote::SwitchState rss = Remote::SwitchState::UNKNOWN;
    bool lb = false;
    bool rb = false;
};

TEST_F(RemoteTest, read_parses_18_bytes_received_all_at_once)
{
    rh = 1;
    rv = -2;
    lh = 3;
    lv = -4;
    keys = 5;
    mx = 6;
    my = 7;
    mz = 8;
    wheel = 9;
    lss = Remote::SwitchState::DOWN;
    rss = Remote::SwitchState::UP;
    lb = false;
    rb = true;

    encodeRemoteData();

    remote.read();

    evaluateRemoteInfo();

    EXPECT_TRUE(remote.isConnected());
}

TEST_F(RemoteTest, read_incomplete_message_thrown_away_connection_status_accurate)
{
    encodeRemoteData();

    // first byte removed, incomplete.
    encodedRemoteData.pop_front();

    remote.read();

    EXPECT_FALSE(remote.isConnected());

    clock.time += 1000;

    encodeRemoteData();

    remote.read();

    EXPECT_TRUE(remote.isConnected());

    clock.time += 1000;

    remote.read();

    EXPECT_FALSE(remote.isConnected());
}

TEST_F(RemoteTest, read_after_timeout_remote_data_reset)
{
    wheel = 100;
    encodeRemoteData();

    remote.read();

    evaluateRemoteInfo();

    clock.time += 1000;

    remote.read();

    wheel = 0;

    evaluateRemoteInfo();
}

TEST_F(RemoteTest, read_partial_data_in_uart_buffer)
{
    wheel = 100;
    encodeRemoteData();
    // remove last byte
    encodedRemoteData.pop_back();

    remote.read();

    EXPECT_FALSE(remote.isConnected());

    encodedRemoteData.push_back((wheel + 1024) >> 8);

    remote.read();

    EXPECT_TRUE(remote.isConnected());

    evaluateRemoteInfo();
}

TEST_F(RemoteTest, read_invalid_joystick_values_errors)
{
    rh = 661;

    encodeRemoteData();

    EXPECT_CALL(drivers.errorController, addToErrorList);

    remote.read();
}
