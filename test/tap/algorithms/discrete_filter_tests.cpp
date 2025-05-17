/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/discrete_filter.hpp"

using namespace tap::algorithms::filter;

TEST(DiscreteFilter, initial_output_is_zero)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0, 0.0, 0.0};
    std::array<float, SIZE> forced{1.0, 0.0, 0.0};
    DiscreteFilter<SIZE> filter(natural, forced);

    EXPECT_NEAR(filter.getLastFiltered(), 0.0, 1e-6);
}

TEST(DiscreteFilter, single_input_response_matches_coefficients)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0f, -1.0f, 0.0f};
    std::array<float, SIZE> forced{0.5f, 0.0f, 0.0f};
    DiscreteFilter<SIZE> filter(natural, forced);

    float out = filter.filterData(1.0f);
    EXPECT_NEAR(out, 0.5, 1e-6);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), out);
}

TEST(DiscreteFilter, double_input_response_matches_coefficients)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0f, -1.0f, 0.0f};
    std::array<float, SIZE> forced{0.5f, 0.0f, 0.0f};
    DiscreteFilter<SIZE> filter(natural, forced);

    float out = filter.filterData(1.0f);
    EXPECT_NEAR(out, 0.5, 1e-6);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), out);
    out = filter.filterData(1.0f);
    EXPECT_NEAR(out, 1.0f, 1e-6);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), out);
}

TEST(DiscreteFilter, repeated_input_updates_internal_state)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0, -0.3, 0.1};
    std::array<float, SIZE> forced{0.1, 0.2, 0.3};
    DiscreteFilter<SIZE> filter(natural, forced);

    float out1 = filter.filterData(1.0);
    float out2 = filter.filterData(1.0);
    float out3 = filter.filterData(1.0);

    // Test that outputs evolve over time (i.e., filter has memory)
    EXPECT_NE(out1, out2);
    EXPECT_NE(out2, out3);
    EXPECT_FLOAT_EQ(out3, filter.getLastFiltered());
}

TEST(DiscreteFilter, zero_input_remains_zero)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0, 0.0, 0.0};
    std::array<float, SIZE> forced{0.1, 0.0, 0.0};
    DiscreteFilter<SIZE> filter(natural, forced);

    for (int i = 0; i < 5; ++i)
    {
        float output = filter.filterData(0.0);
        EXPECT_NEAR(output, 0.0, 1e-6);
    }
}

TEST(DiscreteFilter, filter_resets_properly)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0, -0.3, 0.1};
    std::array<float, SIZE> forced{0.1, 0.2, 0.3};
    DiscreteFilter<SIZE> filter(natural, forced);

    // Apply some input to the filter
    filter.filterData(1.0);
    filter.filterData(1.0);
    filter.filterData(1.0);
    // check that the state has changed
    EXPECT_NE(filter.getLastFiltered(), 0.0f);
    // Reset the filter
    filter.reset();
    // Check that the filter state is reset to zero
    EXPECT_EQ(filter.getLastFiltered(), 0.0f);
}

TEST(DiscreteFilter, handles_step_input)
{
    constexpr uint8_t SIZE = 3;
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};
    DiscreteFilter<SIZE> filter(natural, forced);

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = filter.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), output);
}

TEST(DiscreteFilter, accepts_coefficients_struct)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe;
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};
    coe.naturalResponseCoefficients = natural;
    coe.forcedResponseCoefficients = forced;
    DiscreteFilter<SIZE> filter(coe);

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = filter.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), output);
}

TEST(DiscreteFilter, set_coefficients_struct_works)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    Coefficients<SIZE> coe;
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};
    coe.naturalResponseCoefficients = natural;
    coe.forcedResponseCoefficients = forced;

    filter.setCoefficients(coe);

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = filter.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), output);
}

TEST(DiscreteFilter, set_coefficients_works)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};

    filter.setCoefficients(natural, forced);

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = filter.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(filter.getLastFiltered(), output);
}