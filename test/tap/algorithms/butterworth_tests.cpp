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

#include <algorithm>

#include <gtest/gtest.h>

#include "tap/algorithms/butterworth.hpp"
#include "tap/algorithms/discrete_filter.hpp"

using namespace tap::algorithms;

TEST(S2ZTransform, identity_for_zero_pole)
{
    std::complex<double> s(0, 0);
    double Ts = 0.01;
    auto z = s2z(s, Ts);
    EXPECT_NEAR(1.0, z.real(), 1e-6);
    EXPECT_NEAR(0.0, z.imag(), 1e-6);
}

TEST(S2ZTransform, known_input_output)
{
    std::complex<double> s(-1.0, 0.0);
    double Ts = 0.1;
    auto z = s2z(s, Ts);
    EXPECT_NEAR(0.904761904761905, z.real(), 1e-6);
    EXPECT_NEAR(0, z.imag(), 1e-6);
}

TEST(PolynomialExpansion, real_roots_order_2)
{
    constexpr uint8_t ORDER = 2;
    std::array<std::complex<double>, ORDER> zeros = {
        std::complex<double>(-1.0, 0.0),
        std::complex<double>(-2.0, 0.0)};
    auto coeffs = expandPolynomial<ORDER>(zeros);
    // Expected polynomial: (x + 1)(x + 2) = x^2 + 3x + 2
    EXPECT_NEAR(2.0, coeffs[0], 1e-6);
    EXPECT_NEAR(3.0, coeffs[1], 1e-6);
    EXPECT_NEAR(1.0, coeffs[2], 1e-6);
}

TEST(CalculateScalar, unity_gain)
{
    constexpr uint8_t ORDER = 2;
    std::array<double, ORDER + 1> numerator = {1.0, 2.0, 1.0};    // sum to 4.0
    std::array<double, ORDER + 1> denominator = {0.5, 1.0, 0.5};  // sum to 2.0
    double scalar = calculateScalar<ORDER>(numerator, denominator);
    double expected = 0.5;
    EXPECT_NEAR(expected, scalar, 1e-6);
}

TEST(ButterworthFilter, low_order_filter_coefficients_sum_to_one)
{
    constexpr uint8_t ORDER = 1;
    double wc = 10.0;
    double Ts = 1 / 500.0;
    Butterworth<ORDER> filter(wc, Ts);
    auto num = filter.getForcedResponseCoefficients();
    auto den = filter.getNaturalResponseCoefficients();

    double numSum = 0;
    double denSum = 0;
    for (int i = 0; i <= ORDER; ++i)
    {
        numSum += num[i];
        denSum += den[i];
    }

    // Check DC gain is 1 (numerator and denominator sums are equal)
    EXPECT_NEAR(numSum, denSum, 1e-6);
}

TEST(ButterworthFilter, second_order_filter_has_correct_size)
{
    constexpr uint8_t ORDER = 2;
    double wc = 20.0;
    double Ts = 0.01;
    Butterworth<ORDER> filter(wc, Ts);
    auto num = filter.getForcedResponseCoefficients();
    auto den = filter.getNaturalResponseCoefficients();

    EXPECT_EQ(num.size(), ORDER + 1);
    EXPECT_EQ(den.size(), ORDER + 1);
}

TEST(ButterworthFilter, coefficients_are_what_they_should_be)
{
    static constexpr uint8_t ORDER = 2;
    static constexpr double wc = 10.0;
    static constexpr double Ts = 1 / 500.0;
    static constexpr Butterworth<ORDER> filter(wc, Ts);
    auto num = filter.getForcedResponseCoefficients();
    auto den = filter.getNaturalResponseCoefficients();
    EXPECT_NEAR(num[0], 0.099858678643663 * 1.0e-5, 1e-3);
    EXPECT_NEAR(num[1], 0.199717357287326 * 1.0e-5, 1e-3);
    EXPECT_NEAR(num[2], 0.099858678643663 * 1.0e-5, 1e-3);

    EXPECT_NEAR(den[0], 1, 1e-3);
    EXPECT_NEAR(den[1], -1.971, 1e-3);
    EXPECT_NEAR(den[2], 0.972, 1e-3);
}

struct AttenuationParams
{
    float frequency;
    float max, min = 0;
};

class AttenuationTest : public testing::Test, public testing::WithParamInterface<AttenuationParams>
{
protected:
    static constexpr uint8_t ORDER = 2;
    static constexpr double wc = 10.0;
    static constexpr double Ts = 1 / 500.0;
    static constexpr Butterworth<ORDER> filter{wc, Ts};
};

TEST_P(AttenuationTest, filter_attenuates_properly)
{
    auto nat = filter.getNaturalResponseCoefficients();
    auto force = filter.getForcedResponseCoefficients();

    DiscreteFilter<ORDER + 1> discreteFilter(nat, force);

    float max_val = 0.0f;
    for (int i = 0; i < 10000; i++)
    {
        float val = discreteFilter.filterData(sin(
            GetParam().frequency * (i * Ts)));  // Feed in a sin wave with frequency and amp of 1
        if (i > 5000)
        {
            max_val = std::max(max_val, std::abs(val));
        }
    }
    EXPECT_LT(max_val, GetParam().max);  // Check that the output is attenuated
    EXPECT_GT(max_val, GetParam().min);  // Check that the output is not too attenuated
}

INSTANTIATE_TEST_SUITE_P(
    ButterworthFilter,
    AttenuationTest,
    testing::Values(
        AttenuationParams{.frequency = 1.0, .max = 1.0 + 1e-3, .min = 1.0 - 1e-3},
        AttenuationParams{.frequency = 100.0, .max = 1e-2}));