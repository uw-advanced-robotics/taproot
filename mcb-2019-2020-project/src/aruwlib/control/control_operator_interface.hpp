#ifndef __CONTROL_OPERATOR_INTERFACE_HPP__
#define __CONTROL_OPERATOR_INTERFACE_HPP__

#include "src/aruwlib/algorithms/linear_interpolation.hpp"

namespace aruwlib
{

namespace control
{

class ControlOperatorInterface {
 public:
    // Returns the value used for chassis movement forward and backward, between -1 and 1
    static float getChassisXInput();

    // Returns the value used for chassis movement side to side, between -1 and 1
    static float getChassisYInput();

    // Returns the value used for chassis rotation, between -1 and 1
    static float getChassisRInput();

    // Returns the value used for turret yaw rotation, between about -1 and 1
    // this value can be greater or less than (-1, 1) since the mouse input has no
    // clear lower and upper bound
    static float getTurretYawInput();

    // Returns the value used for turret pitch rotation, between about -1 and 1
    // this value can be greater or less than (-1, 1) since the mouse input has no
    // clear lower and upper bound
    static float getTurretPitchInput();

    static float getSentinelSpeedInput();

 private:
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / 1000.0f);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / 1000.0f);

    static constexpr float USER_STICK_SENTINEL_DRIVE_SCALAR = 5000.0f;

    static uint32_t prevUpdateCounterX;
    static uint32_t prevUpdateCounterY;
    static uint32_t prevUpdateCounterZ;

    static aruwlib::algorithms::LinearInterpolation chassisXInput;
    static aruwlib::algorithms::LinearInterpolation chassisYInput;
    static aruwlib::algorithms::LinearInterpolation chassisRInput;
};

}  // namespace control

}  // namespace aruwlib

#endif
