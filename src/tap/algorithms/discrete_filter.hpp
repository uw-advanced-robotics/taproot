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
        std::array<double, SIZE> &naturalResponseCoefficients,
        std::array<double, SIZE> &forcedResponseCoefficients)
        : naturalResponseCoefficients(naturalResponseCoefficients),
          forcedResponseCoefficients(forcedResponseCoefficients)
    {
    }

    double filterData(double dat)
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

        double sum = 0;
        for (int i = 0; i < SIZE; i++)
        {
            sum += forcedResponseCoefficients[i] * forcedResponse[i];
        }

        for (int i = 0; i < SIZE - 1; i++)
        {
            sum -= naturalResponseCoefficients[i + 1] * naturalResponse[i];
        }

        sum /= naturalResponseCoefficients[0];

        for (int i = SIZE - 1; i >= 0; i--)
        {
            if (i == 0)
            {
                naturalResponse[i] = sum;
                break;
            }
            naturalResponse[i] = naturalResponse[i - 1];
        }

        return naturalResponse[0];
    }

    double getLastFiltered() { return naturalResponse[0]; }

private:
    std::array<double, SIZE> naturalResponseCoefficients;
    std::array<double, SIZE> forcedResponseCoefficients;
    std::array<double, SIZE> naturalResponse;
    std::array<double, SIZE> forcedResponse;
};

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_DISCRETE_FILTER_HPP_