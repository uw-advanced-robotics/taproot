#ifndef __TURRET_PID_HPP__
#define __TURRET_PID_HPP__

#include <cstdint>

#include <aruwlib/algorithms/extended_kalman.hpp>

namespace aruwsrc
{
namespace algorithms
{
class TurretPid
{
public:
    TurretPid(
        float kp,
        float ki,
        float kd,
        float maxICumulative,
        float maxOutput,
        float tQDerivativeKalman,
        float tRDerivativeKalman,
        float tQProportionalKalman,
        float tRProportionalKalman)
        : kp(kp),
          ki(ki),
          kd(kd),
          maxICumulative(maxICumulative),
          maxOutput(maxOutput),
          proportionalKalman(tQProportionalKalman, tRProportionalKalman),
          derivativeKalman(tQDerivativeKalman, tRDerivativeKalman)
    {
    }

    float runController(float error, float rotationalSpeed);

    float runControllerDerivateError(float error, float dt);

    float getOutput();

    void reset();

private:
    // gains and constants, to be set by the user
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxICumulative = 0.0f;
    float maxOutput = 0.0f;

    // while these could be local, debugging pid is much easier if they are not
    float currErrorP = 0.0f;
    float currErrorI = 0.0f;
    float currErrorD = 0.0f;
    float output = 0.0f;

    // if you run the controller without inputting a rotationalSpeed, we find the
    // derivative ourselves
    uint32_t previousTimestamp = 0;

    aruwlib::algorithms::ExtendedKalman proportionalKalman;
    aruwlib::algorithms::ExtendedKalman derivativeKalman;
};

}  // namespace algorithms

}  // namespace aruwsrc

#endif
