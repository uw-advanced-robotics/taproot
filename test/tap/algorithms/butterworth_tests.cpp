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

using namespace tap::algorithms::filter;

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

TEST(EvaluateFrequencyResponse, unity_gain)
{
    constexpr int ORDER = 2;
    constexpr double Ts = 1 / 500.0;
    /*
     * Coefficients for a lowpass filter with a wc of 1 and a Ts of 1/500.
     * Note: it's in backwards order as everything within Butterworth is calculated in reverse from
     * delivered. Also, it's actually necessary to have this amount of digits; 6 digits of precision
     * is not enough.
     */
    std::array<double, ORDER + 1> numerator = {
        9.985875464857408e-07,
        1.997175092971482e-06,
        9.985875464857408e-07};
    std::array<double, ORDER + 1> denominator = {9.971755689727204e-01, -1.997171574622534, 1};
    std::complex<double> resp = evaluateFrequencyResponse<ORDER>(numerator, denominator, 0, Ts);
    double scalar = complexAbs(resp);
    double expected = 1;
    EXPECT_NEAR(expected, scalar, 1e-6);
}

TEST(ButterworthFilter, low_order_filter_coefficients_sum_to_one)
{
    constexpr uint8_t ORDER = 1;
    double wc = 10.0;
    double Ts = 1 / 500.0;
    auto coefs = butterworth<ORDER>(wc, Ts);

    double numSum = 0;
    double denSum = 0;
    for (int i = 0; i <= ORDER; ++i)
    {
        numSum += coefs.naturalResponseCoefficients[i];
        denSum += coefs.forcedResponseCoefficients[i];
    }

    // Check DC gain is 1 (numerator and denominator sums are equal)
    EXPECT_NEAR(numSum, denSum, 1e-6);
}

TEST(ButterworthFilter, second_order_filter_has_correct_size)
{
    constexpr uint8_t ORDER = 2;
    double wc = 20.0;
    double Ts = 0.01;
    auto coe = butterworth<ORDER>(wc, Ts);

    EXPECT_EQ(coe.forcedResponseCoefficients.size(), ORDER + 1);
    EXPECT_EQ(coe.naturalResponseCoefficients.size(), ORDER + 1);
}

TEST(ButterworthFilter, coefficients_are_what_they_should_be)
{
    static constexpr uint8_t ORDER = 2;
    static constexpr double wc = 10.0;
    static constexpr double Ts = 1 / 500.0;
    auto coe = butterworth<ORDER>(wc, Ts);
    EXPECT_NEAR(coe.forcedResponseCoefficients[0], 0.099858678643663 * 1.0e-5, 1e-3);
    EXPECT_NEAR(coe.forcedResponseCoefficients[1], 0.199717357287326 * 1.0e-5, 1e-3);
    EXPECT_NEAR(coe.forcedResponseCoefficients[2], 0.099858678643663 * 1.0e-5, 1e-3);

    EXPECT_NEAR(coe.naturalResponseCoefficients[0], 1, 1e-3);
    EXPECT_NEAR(coe.naturalResponseCoefficients[1], -1.971, 1e-3);
    EXPECT_NEAR(coe.naturalResponseCoefficients[2], 0.972, 1e-3);
}

struct AttenuationParams
{
    float frequency;
    float max, min = 0;
};

template <FilterType Type>
class AttenuationTest : public testing::Test, public testing::WithParamInterface<AttenuationParams>
{
protected:
    static constexpr uint8_t ORDER = 2;
    static constexpr double wc = 10.0;
    static constexpr double wh = 100.0;
    static constexpr double Ts = 1 / 500.0;
    static constexpr Coefficients coe = butterworth<ORDER, Type, double>(wc, Ts, wh);
};

// clang-format off
#define ATTENUATION_TEST(Type) \
    using Type##AttenuationTest = AttenuationTest<Type>; \
    TEST_P(Type##AttenuationTest, filter_attenuates_properly) \
    { \
        DiscreteFilter<getNumCoefficients(ORDER, Type), double> \
        discreteFilter(coe); \
        float max_val = 0.0f; \
        constexpr int simulation_points = 50000; \
        for (int i = 0; i < simulation_points; i++) \
        { \
            float val = discreteFilter.filterData(sin( \
                GetParam().frequency * (i * Ts)));  /* Feed in a sin wave with freq, amp = 1 */\
            if (i > simulation_points - ((2*M_PI)/GetParam().frequency/Ts + 100)) \
            { \
                max_val = std::max(max_val, std::abs(val)); \
            } \
        } \
        EXPECT_LT(max_val, GetParam().max);  /* Check that the output is attenuated */ \
        EXPECT_GT(max_val, GetParam().min);  /* Check that the output is not too attenuated */ \
    } \
    TEST_P(Type##AttenuationTest, runtime_matches_constexpr) \
    { \
        Coefficients coef = butterworth<ORDER, Type, double>(wc, Ts, wh); \
        auto nat = coef.naturalResponseCoefficients; \
        auto force = coef.forcedResponseCoefficients; \
        auto nat_constexpr = coe.naturalResponseCoefficients; \
        auto force_constexpr = coe.forcedResponseCoefficients; \
        for (size_t i = 0; i < getNumCoefficients(ORDER, Type); ++i) \
        { \
            EXPECT_NEAR(nat[i], nat_constexpr[i], 1e-6); \
            EXPECT_NEAR(force[i], force_constexpr[i], 1e-6); \
        } \
    } \
// clang-format on

ATTENUATION_TEST(LOWPASS)
INSTANTIATE_TEST_SUITE_P(
    ButterworthFilter,
    LOWPASSAttenuationTest,
    testing::Values(
        AttenuationParams{.frequency = 1.0, .max = 1.0 + 1e-3, .min = 1.0 - 1e-3},
        AttenuationParams{.frequency = 100.0, .max = 1e-2}));

ATTENUATION_TEST(HIGHPASS)
INSTANTIATE_TEST_SUITE_P(
    ButterworthFilter,
    HIGHPASSAttenuationTest,
    testing::Values(
        AttenuationParams{.frequency = 1.0, .max = 1e-2},
        AttenuationParams{.frequency = 100.0, .max = 1.0 + 1e-3, .min = 1.0 - 1e-3}));

ATTENUATION_TEST(BANDPASS)
INSTANTIATE_TEST_SUITE_P(
    ButterworthFilter,
    BANDPASSAttenuationTest,
    testing::Values(
        AttenuationParams{.frequency = 1.0, .max = 1e-2},
        AttenuationParams{.frequency = 30.0, .max = 1.0 + 1e-2, .min = 1.0 - 1e-2},
        AttenuationParams{.frequency = 50.0, .max = 1.0 + 1e-2, .min = 1.0 - 1e-2},
        AttenuationParams{.frequency = 1000.0, .max = 1e-2}));

ATTENUATION_TEST(BANDSTOP)
INSTANTIATE_TEST_SUITE_P(
    ButterworthFilter,
    BANDSTOPAttenuationTest,
    testing::Values(
        AttenuationParams{.frequency = 1, .max = 1.0 + 1e-2, .min = 1.0 - 1e-2},
        AttenuationParams{.frequency = 30.0, .max = 1e-2},
        AttenuationParams{.frequency = 40.0, .max = 1e-1},
        AttenuationParams{.frequency = 1000.0, .max = 1.0 + 1e-2, .min = 1.0 - 1e-2}));
