#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "aruwlib/algorithms/linear_interpolation.hpp"

namespace aruwlib
{
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 */
class ControlOperatorInterface
{
public:
    ControlOperatorInterface() = default;
    ControlOperatorInterface(const ControlOperatorInterface &) = delete;
    ControlOperatorInterface &operator=(const ControlOperatorInterface &) = default;

    ///< @return the value used for chassis movement forward and backward, between -1 and 1.
    float getChassisXInput();

    ///< @return the value used for chassis movement side to side, between -1 and 1.
    float getChassisYInput();

    ///< @return the value used for chassis rotation, between -1 and 1.
    float getChassisRInput();

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    float getTurretYawInput();

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    float getTurretPitchInput();

    /**
     * @returns the value used for sentiel drive speed, between
     *      [-USER_STICK_SENTINEL_DRIVE_SCALAR, USER_STICK_SENTINEL_DRIVE_SCALAR].
     */
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
};  // class ControlOperatorInterface

}  // namespace control

}  // namespace aruwlib

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_
