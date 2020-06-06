#ifndef __CONTROL_OPERATOR_INTERFACE_HPP__
#define __CONTROL_OPERATOR_INTERFACE_HPP__

#include "aruwlib/algorithms/linear_interpolation.hpp"

namespace aruwlib
{
namespace control
{
class ControlOperatorInterface
{
public:
    ControlOperatorInterface() = default;
    ControlOperatorInterface(const ControlOperatorInterface &) = delete;
    ControlOperatorInterface &operator=(const ControlOperatorInterface &) = default;

    // Returns the value used for chassis movement forward and backward, between -1 and 1
    float getChassisXInput();

    // Returns the value used for chassis movement side to side, between -1 and 1
    float getChassisYInput();

    // Returns the value used for chassis rotation, between -1 and 1
    float getChassisRInput();

    // Returns the value used for turret yaw rotation, between about -1 and 1
    // this value can be greater or less than (-1, 1) since the mouse input has no
    // clear lower and upper bound
    float getTurretYawInput();

    // Returns the value used for turret pitch rotation, between about -1 and 1
    // this value can be greater or less than (-1, 1) since the mouse input has no
    // clear lower and upper bound
    float getTurretPitchInput();

    float getSentinelSpeedInput();

private:
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / 1000.0f);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / 1000.0f);

    static constexpr float USER_STICK_SENTINEL_DRIVE_SCALAR = 5000.0f;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterZ = 0;

    aruwlib::algorithms::LinearInterpolation chassisXInput;
    aruwlib::algorithms::LinearInterpolation chassisYInput;
    aruwlib::algorithms::LinearInterpolation chassisRInput;
};

}  // namespace control

}  // namespace aruwlib

#endif
