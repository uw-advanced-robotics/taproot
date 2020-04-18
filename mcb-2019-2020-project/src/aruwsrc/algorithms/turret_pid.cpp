#include "turret_pid.hpp"
#include <aruwlib/algorithms/math_user_utils.hpp>

using namespace aruwlib::algorithms;

namespace aruwsrc
{

namespace algorithms
{

float TurretPid::runController(float error, float errorDerivative)
{
    // p
    currErrorP = kp * proportionalKalman.filterData(error);
    // i
    currErrorI = limitVal<float>(currErrorI + ki * proportionalKalman.getLastFiltered(),
        -maxICumulative, maxICumulative);
    // d
    currErrorD = -kd * derivativeKalman.filterData(errorDerivative);
    // total
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
    return output;
}

float TurretPid::runControllerDerivateError(float error, float dt)
{
    float errorDerivative = error / dt;
    previousTimestamp = modm::Clock::now().getTime();
    return runController(error, errorDerivative);
}

float TurretPid::getOutput()
{
    return output;
}

void TurretPid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->derivativeKalman.reset();
    this->proportionalKalman.reset();
}

}  // namespace algorithms

}  // namespace aruwsrc
