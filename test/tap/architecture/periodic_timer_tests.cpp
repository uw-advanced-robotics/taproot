/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/clock.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/test_macros.hpp"

using namespace tap;
using namespace testing;
using namespace tap::arch;

// Assuming Timeout is templated or configured to use the Clock stub's time function
// (e.g., tap::arch::clock::getTimeMicroseconds) for testing purposes.
class TimeoutTest : public Test
{
protected:
    TimeoutTest() {}

    void SetUp() override
    {
        // Ensure clock starts at a known 0 state before each test
        clock.time = 0;
    }

    clock::ClockStub clock;
};

TEST_F(TimeoutTest, normal_expiration_no_wrap)
{
    clock.time = 100;
    Timeout timeout(50);  // 50 tick timeout

    EXPECT_FALSE(timeout.isExpired());
    EXPECT_EQ(50, timeout.timeRemaining());

    clock.time = 149;
    EXPECT_FALSE(timeout.isExpired());
    EXPECT_EQ(1, timeout.timeRemaining());

    clock.time = 150;
    EXPECT_TRUE(timeout.isExpired());
    EXPECT_EQ(0, timeout.timeRemaining());
}

TEST_F(TimeoutTest, expiration_across_32bit_boundary)
{
    // Start 10 ticks before the 32-bit wrap around
    clock.time = UINT32_MAX - 10;

    // 20 tick timeout. Expire time should mathematically be 9.
    Timeout timeout(20);

    EXPECT_FALSE(timeout.isExpired());

    clock.time = UINT32_MAX;
    EXPECT_FALSE(timeout.isExpired());

    clock.time = 0;  // Wraparound occurs here
    EXPECT_FALSE(timeout.isExpired());

    clock.time = 8;
    EXPECT_FALSE(timeout.isExpired());
    EXPECT_EQ(1, timeout.timeRemaining());

    clock.time = 9;
    EXPECT_TRUE(timeout.isExpired());
    EXPECT_EQ(0, timeout.timeRemaining());
}

TEST_F(TimeoutTest, long_delay_past_signed_integer_limit)
{
    clock.time = 100;
    Timeout timeout(10);  // 10 tick timeout

    EXPECT_FALSE(timeout.isExpired());

    // Simulate a delay of 45 minutes
    uint32_t forty_five_minutes_us = 45UL * 60UL * 1000UL * 1000UL;

    clock.time += forty_five_minutes_us;

    EXPECT_TRUE(timeout.isExpired());
}

TEST_F(TimeoutTest, time_remaining_across_boundary)
{
    clock.time = UINT32_MAX - 50;
    Timeout timeout(100);

    // Current time is before wrap, expire time is 49 (after wrap)
    EXPECT_FALSE(timeout.isExpired());
    EXPECT_EQ(100, timeout.timeRemaining());

    clock.time = UINT32_MAX - 10;
    EXPECT_EQ(60, timeout.timeRemaining());

    clock.time = 10;  // Wrapped
    EXPECT_EQ(39, timeout.timeRemaining());

    clock.time = 49;  // Expired
    EXPECT_TRUE(timeout.isExpired());
    EXPECT_EQ(0, timeout.timeRemaining());
}

TEST_F(TimeoutTest, increment_expire_time_across_boundary)
{
    clock.time = UINT32_MAX - 50;
    Timeout timeout(20);  // Initial expire time: UINT32_MAX - 30

    EXPECT_FALSE(timeout.isExpired());

    // Increment pushes the expire time across the uint32_t boundary to 20
    timeout.incrementExpireTime(50);

    clock.time = UINT32_MAX - 10;
    EXPECT_FALSE(timeout.isExpired());

    clock.time = 10;  // Wrapped, but not yet expired
    EXPECT_FALSE(timeout.isExpired());
    EXPECT_EQ(10, timeout.timeRemaining());

    clock.time = 20;  // Reached the new incremented expiration
    EXPECT_TRUE(timeout.isExpired());
}

TEST_F(TimeoutTest, restart_resets_wrap_state_correctly)
{
    clock.time = UINT32_MAX - 5;
    Timeout timeout(10);  // Wraps, expire is 4

    EXPECT_FALSE(timeout.isExpired());

    clock.time = 10;
    EXPECT_TRUE(timeout.isExpired());

    // Restart the timer post-wrap
    timeout.restart(20);  // New expire is 30, no wrapping relative to current time (10)

    EXPECT_FALSE(timeout.isExpired());

    clock.time = 25;
    EXPECT_FALSE(timeout.isExpired());

    clock.time = 30;
    EXPECT_TRUE(timeout.isExpired());
}