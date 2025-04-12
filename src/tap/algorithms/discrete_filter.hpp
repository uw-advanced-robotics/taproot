
/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington
 * <robomstr@uw.edu>
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
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>;.
 */

#ifndef TAPROOT_DISCRETE_FILTER_HPP_
#define TAPROOT_DISCRETE_FILTER_HPP_

#include <array>
#include <cstdint>

namespace tap
{
namespace algorithms
{
template <uint8_t SIZE>
class DiscreteFilter
{
public:
    DiscreteFilter(
        std::array<float, SIZE> &naturalResponseCoefficients, // a
        std::array<float, SIZE> &forcedResponseCoefficients)  // b
        : naturalResponseCoefficients(naturalResponseCoefficients),
          forcedResponseCoefficients(forcedResponseCoefficients)
    {
        // Fill with zeros to ensure that if getLastFiltered is called
        // before filterData, it returns 0.0
        naturalResponse.fill(0.0f);
        forcedResponse.fill(0.0f);
    }

    /**
     * @brief Filters the input data using the finite difference equation.
     * @param [in] dat The input data to be filtered.
     * @return The filtered output data.
     *
     * This function implements a discrete-time filter using the finite difference equation.
     * It updates the internal state of the filter based on the input data and returns the
     * filtered output.
     *
     * \f$ y(n)=\frac{1}{a_{0}}\left[\sum_{\kappa=0}^{M} b_{\kappa}x(n-\kappa)-\sum_{k=1}^{N}
     * a_{k}y(n-k)\right]. \qquad{(2)} \f$
     *
     */
    float filterData(float dat)
    {
        for (int i = SIZE - 1; i >= 0; i--)
        {
            if (i == 0)
            {
                forcedResponse[i] = dat;
                break;
            }
            forcedResponse[i] = forcedResponse[i - 1];
        }

        float sum = 0;
        // Sum of forced response coefficients multiplied by the forced response X(n-k)
        // (previous input data)
        for (int i = 0; i < SIZE; i++)
        {
            sum += forcedResponseCoefficients[i] * forcedResponse[i];
        }
        // Sum of natural response coefficients multiplied by the natural response Y(n-k)
        // (previous output data)
        for (int i = 0; i < SIZE - 1; i++)
        {
            sum -= naturalResponseCoefficients[i + 1] * naturalResponse[i];
        }
        // Apply the 1/a_0 scaling to the output
        sum /= naturalResponseCoefficients[0];

        // Shift the natural response array to make room for the new output
        std::rotate(naturalResponse.rbegin(), naturalResponse.rbegin() + 1, naturalResponse.rend());
        naturalResponse[0] = sum;

        return naturalResponse[0];
    }

    float getLastFiltered() { return naturalResponse[0]; }

private:
    std::array<float, SIZE> naturalResponseCoefficients;
    std::array<float, SIZE> forcedResponseCoefficients;
    std::array<float, SIZE> naturalResponse;
    std::array<float, SIZE> forcedResponse;
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_DISCRETE_FILTER_HPP_