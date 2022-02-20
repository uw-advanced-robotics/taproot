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

#include <gtest/gtest.h>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/profiler.hpp"
#include "tap/drivers.hpp"

using namespace tap;
using namespace tap::arch;

#define SETUP_TEST()   \
    clock::ClockStub clock;\
    Drivers drivers;   \
    Profiler profiler(&drivers);

TEST(Profiler, push_pop_normal_usage_no_errors)
{
    SETUP_TEST();

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(0);

    profiler.pop(profiler.push("hi"));
}

TEST(Profiler, push_returns_same_key_if_profiler_same)
{
    SETUP_TEST();

    const char* hi = "hi";

    EXPECT_EQ(profiler.push(hi), profiler.push(hi));
}

TEST(Profiler, pop_without_push_errors)
{
    SETUP_TEST();

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);

    profiler.pop(0);
}

TEST(Profiler, pop_without_previous_push_of_same_profile_errors)
{
    SETUP_TEST();

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(2);

    profiler.push("Hi");

    profiler.pop(1);
    profiler.pop(2);
}

TEST(Profiler, after_push_getData_valid)
{
    SETUP_TEST();

    std::size_t key = profiler.push("hi");
    EXPECT_EQ("hi", std::string(profiler.getData(key).name));
}

TEST(Profiler, getData_with_time_btwn_push_pop_populates_min_max_avg)
{
    SETUP_TEST();

    std::size_t key = profiler.push("hi");

    clock.time = 12;  // 12 ms == 12'000 us

    profiler.pop(key);

    Profiler::ProfilerData data = profiler.getData(key);

    EXPECT_EQ(12'000, data.min);
    EXPECT_EQ(12'000, data.max);
    EXPECT_EQ(algorithms::lowPassFilter(0, 12'000, Profiler::AVG_LOW_PASS_ALPHA), data.avg);
}

TEST(Profiler, getData_multiple_push_pops_chooses_correct_min_max)
{
    SETUP_TEST();

    const char* hi = "hi";

    std::size_t key = profiler.push(hi);
    clock.time = 1;
    profiler.pop(key);

    key = profiler.push(hi);
    clock.time = 3;
    profiler.pop(key);

    key = profiler.push(hi);
    clock.time = 6;
    profiler.pop(key);

    Profiler::ProfilerData data = profiler.getData(key);

    EXPECT_EQ(1000, data.min);
    EXPECT_EQ(3000, data.max);
}

TEST(Profiler, push_big_batch_insertion)
{
    SETUP_TEST();

    EXPECT_CALL(drivers.errorController, addToErrorList).Times(1);

    std::string strs[Profiler::MAX_PROFILED_ELEMENTS];

    for (std::size_t i = 0; i < Profiler::MAX_PROFILED_ELEMENTS; i++)
    {
        strs[i] = std::to_string(i);
        profiler.push(strs[i].c_str());
    }

    // this will cause the error
    profiler.push("hi");
}

// redeclare profile macro s.t. we can test it even if profiling is turned off
#undef PROFILE
#define PROFILE(profiler, func, params) \
    do                                  \
    {                                   \
        int key = profiler.push(#func); \
        func params;                    \
        profiler.pop(key);              \
    } while (0);

void testFunc(clock::ClockStub &clock) { clock.time += 1; }

TEST(Profiler, profile_macro)
{
    // make sure if we have a variable called "key" in function scope the PROFILE macro still works
    int key = 10;

    SETUP_TEST();

    PROFILE(profiler, testFunc, (clock));

    EXPECT_EQ(10, key);

    Profiler::ProfilerData data = profiler.getData(0);

    EXPECT_EQ(1000, data.min);
    EXPECT_EQ(1000, data.max);
}
