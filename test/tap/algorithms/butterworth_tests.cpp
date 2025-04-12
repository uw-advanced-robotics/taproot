#include <gtest/gtest.h>

#include "tap/algorithms/butterworth.hpp"

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
    constexpr uint8_t ORDER = 2;
    double wc = 10.0;
    double Ts = 1 / 500.0;
    Butterworth<ORDER> filter(wc, Ts);
    auto num = filter.getForcedResponseCoefficients();
    auto den = filter.getNaturalResponseCoefficients();
    EXPECT_NEAR(num[0], 0.099858678643663 * 1.0e-5, 1e-3);
    EXPECT_NEAR(num[1], 0.199717357287326 * 1.0e-5, 1e-3);
    EXPECT_NEAR(num[2], 0.099858678643663 * 1.0e-5, 1e-3);

    EXPECT_NEAR(den[0], 0.972, 1e-3);
    EXPECT_NEAR(den[1], -1.971, 1e-3);
    EXPECT_NEAR(den[2], 1, 1e-3);
}