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

#include <iomanip>
#include <iostream>

#include <gtest/gtest.h>

#include "tap/algorithms/butterworth.hpp"
#include "tap/algorithms/discrete_filter.hpp"

using namespace tap::algorithms::filter;

template <typename Cascade>
float runFilterMaxValue(Cascade& c, float freq, float Ts)
{
    float max_val = 0.0f;
    constexpr int N = 5000;

    for (int i = 0; i < N; ++i)
    {
        float data = sin(freq * (i * Ts));
        float out = c.filterData(data);

        if (i > N - 1000)
        {
            max_val = std::max(max_val, std::abs(out));
        }
    }
    return max_val;
}

template <typename C>
void compareCascades(C& c1, C& c2, float freq, float Ts)
{
    auto a1 = runFilterMaxValue(c1, freq, Ts);
    auto a2 = runFilterMaxValue(c2, freq, Ts);

    EXPECT_NEAR(a1, a2, 1e-3);
    EXPECT_NEAR(c1.getLastFiltered(), c2.getLastFiltered(), 1e-3);
}

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

TEST(CASCADEFILTER, cascade_two_filters)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};

    filter.setCoefficients(natural, forced);
    auto cascade = filter * filter;

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = cascade.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(cascade.getLastFiltered(), output);
}

TEST(CASCADEFILTER, cascade_size_works)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};

    filter.setCoefficients(natural, forced);
    auto cascade = filter * filter * filter;

    EXPECT_EQ(cascade.size(), 3);
}

TEST(CASCADEFILTER, indexing_operator_works)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    std::array<float, SIZE> natural{1.0, -0.5, 0.25};
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};

    filter.setCoefficients(natural, forced);
    auto cascade = filter * filter * filter;

    // Check that we can access the individual filters via the indexing operator
    cascade[0].setCoefficients(natural, forced);
    cascade[1].setCoefficients(natural, forced);
    cascade[2].setCoefficients({1, 0, 0}, {0, 0, 0});

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = cascade.filterData(1.0);
    }

    // The filter output should be forced to zero by the last filter
    EXPECT_FLOAT_EQ(output, 0.0);
    EXPECT_FLOAT_EQ(cascade.getLastFiltered(), output);
}

TEST(CASCADEFILTER, index_at_runtime)
{
    constexpr uint8_t SIZE = 3;
    Coefficients<SIZE> coe_empty{{0, 0, 0}, {0, 0, 0}};
    DiscreteFilter<SIZE> filter(coe_empty);
    std::array<float, SIZE> forced{0.2, 0.1, 0.05};

    // Create and empty filter
    auto cascade = filter * filter * filter;

    for (size_t i = 0; i < cascade.size(); ++i)
    {
        // Fill filter with slightly different natural coefficients
        cascade[i].setCoefficients({1.0f, -.5f, .25f + i * 0.1f}, forced);
    }

    float output = 0.0;
    for (int i = 0; i < 1e3; ++i)
    {
        output = cascade.filterData(1.0);
    }

    // The filter output should settle to a non-zero value
    EXPECT_GT(output, 0.0);
    EXPECT_FLOAT_EQ(cascade.getLastFiltered(), output);
}

TEST(CASCADEFILTER, all_multiplication_permutations_work)
{
    constexpr double wc = 10.0;
    constexpr double Ts = 1 / 500.0;
    float frequency = 100.0;

    DiscreteFilter<2> f1(butterworth<1, LOWPASS>(wc, Ts));
    DiscreteFilter<2> f2(butterworth<1, LOWPASS>(wc, Ts));
    DiscreteFilter<2> f3(butterworth<1, LOWPASS>(wc, Ts));

    // Reference manually–chained cascade
    auto ref = CascadeFilter(f1, f2, f3);

    // Test all equivalent permutations:
    auto A = f1 * f2 * f3;        // filter * filter * filter
    auto B = (f1 * f2) * f3;      // (cascade) * filter
    auto C = f1 * (f2 * f3);      // filter * (cascade)
    auto D = (f1 * f2) * (f3);    // cascade * filter as cascade
    auto E = (f1) * (f2 * f3);    // filter * cascade
    auto F = (f1 * (f2)) * (f3);  // weird parentheses but valid case

    compareCascades(ref, A, frequency, Ts);
    compareCascades(ref, B, frequency, Ts);
    compareCascades(ref, C, frequency, Ts);
    compareCascades(ref, D, frequency, Ts);
    compareCascades(ref, E, frequency, Ts);
    compareCascades(ref, F, frequency, Ts);
}