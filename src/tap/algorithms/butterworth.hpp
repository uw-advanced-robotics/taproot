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

#ifndef TAPROOT_BUTTERWORTH_HPP_
#define TAPROOT_BUTTERWORTH_HPP_
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include "modm/math/geometry/angle.hpp"

namespace tap
{
namespace algorithms
{
/**
 * used to transform poles from the laplace domain to the
 * Z domain for descrete time using the bilinear transform
 *
 * @param [in] s a pole or zero from the laplace domain
 * @param [in] Ts the sample time
 * @return a complex number corresponding to the Z domain location
 */
constexpr std::complex<double> s2z(std::complex<double> s, double Ts)
{
    return (1.0 + (Ts / 2) * s) / (1.0 - (Ts / 2) * s);
}

/**
 * used to multiply out a series of zeros to obtain a list of coefficents
 *
 * @param [in] zeros a vector of complex poles or zeros to multiply out, works
 * for any polynomial
 *
 * @return a vector of coefficients for the polynomial with the 0th index being the
 * constant term and the last index being the leading coefficient
 * @note the coefficients are returned as doubles, the imaginary part of the
 * complex number is ignored
 */
template <uint8_t ORDER>
constexpr std::array<double, ORDER + 1> expandPolynomial(
    std::array<std::complex<double>, ORDER> zeros)
{
    std::array<std::complex<double>, ORDER + 1> coefficients{
        std::complex<double>(1.0, 0.0)};  // Start with 1

    for (size_t i = 0; i < ORDER; i++)
    {
        // Multiply current polynomial by (x - zero)
        std::array<std::complex<double>, ORDER + 1> newCoefficients{std::complex<double>(0.0, 0.0)};
        for (size_t j = 0; j < i + 1; j++)
        {
            newCoefficients[j] -=
                coefficients[j] * zeros[i];             // Multiply current coefficient by zero
            newCoefficients[j + 1] += coefficients[j];  // Shift current coefficient
        }
        coefficients.swap(newCoefficients);
    }
    std::array<double, ORDER + 1> stripped_coefficients{};
    // Extract the real part of each coefficient, the imaginary appears to be an
    // artifact of double
    for (size_t i = 0; i < ORDER + 1; i++)
    {
        stripped_coefficients[i] = coefficients[i].real();
    }

    return stripped_coefficients;
}

//* @brief calculates the scalar to apply to the numerator to make the DC gain
//* equal to 1
//* @param [in] numerator the coefficients of the numerator polynomial
//* @param [in] denominator the coefficients of the denominator polynomial
//* @return the scalar to apply to the numerator polynomial

template <uint8_t ORDER>
constexpr double calculateScalar(
    std::array<double, ORDER + 1> numerator,
    std::array<double, ORDER + 1> denominator)
{
    double denResult = 0;
    double numResult = 0;

    for (size_t i = 0; i < ORDER + 1; i++)
    {
        denResult += denominator[i];
        numResult += numerator[i];
    }

    // den is over num in the result as it is in the form 1/(num/den) to apply a
    // scalar for the gain
    return static_cast<double>((denResult / numResult));
}

template <uint8_t ORDER>
class Butterworth
{
public:
    /**
     * Initializes a butterworth filter with a given order, cuttoff frequency, and
     * sample time
     *
     * @note make sure to understand what the a given order affects before
     * implementing, mainly the potential phase delays
     *
     * @param[in] wc the cutoff frequency, roughly the frequency where attenuation
     * begins.
     * @param[in] n the order of the Butterworth, higher orders mean a sharper
     * droppoff in frequency.
     * @param[in] Ts the sample time in seconds, for example at 500hz the value
     * should be 0.02
     */
    constexpr Butterworth(double wc, double Ts)
        : naturalResponseCoefficients(),
          forcedResponseCoefficients()
    {
        int n = ORDER;
        // Warp frequency for bilinear transform
        wc = (2 / Ts) * std::atan(wc * (Ts / 2));

        // generate poles for butterworth filter
        std::array<std::complex<double>, ORDER> poles;
        for (int k = 0; k < static_cast<int>(n); ++k)
        {
            double theta = M_PI * (2.0 * k + 1) / (2.0 * n) + M_PI / 2;
            std::complex<double> pole = std::complex<double>(wc * cos(theta), wc * sin(theta));
            poles[k] = pole;
        }

        // Preform the z transform for each pole
        std::array<std::complex<double>, ORDER> zPoles;
        for (int i = 0; i < ORDER; ++i)
        {
            zPoles[i] = s2z(poles[i], Ts);
        }

        // zeros calculation, for a butterworth all zeros in the Z domain are -1
        std::complex<double> zZero(-1, 0);
        std::array<std::complex<double>, ORDER> zZeros;
        zZeros.fill(zZero);

        // Calculate polynomial coefficients
        auto forcedResponseCoefficients = expandPolynomial<ORDER>(zZeros);
        auto naturalResponseCoefficients = expandPolynomial<ORDER>(zPoles);

        // Calculate and apply the scalar so the DC gain is 1
        double scale_factor =
            calculateScalar<ORDER>(forcedResponseCoefficients, naturalResponseCoefficients);

        for (size_t i = 0; i < ORDER + 1; i++)
        {
            forcedResponseCoefficients[i] *= scale_factor;
        }

        for (size_t i = 0; i < ORDER + 1; i++)
        {
            this->naturalResponseCoefficients[ORDER - i] = naturalResponseCoefficients[i];
            this->forcedResponseCoefficients[ORDER - i] = forcedResponseCoefficients[i];
        }
    }

    std::array<float, ORDER + 1> getNaturalResponseCoefficients() const
    {
        return naturalResponseCoefficients;
    }
    std::array<float, ORDER + 1> getForcedResponseCoefficients() const
    {
        return forcedResponseCoefficients;
    }

private:
    std::array<float, ORDER + 1> naturalResponseCoefficients;

    std::array<float, ORDER + 1> forcedResponseCoefficients;
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_BUTTERWORTH_HPP_