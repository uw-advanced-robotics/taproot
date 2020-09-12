/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "aruwlib/algorithms/linear_interpolation.hpp"

#include "mock_macros.hpp"

namespace aruwlib
{
class Drivers;
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
    ControlOperatorInterface(Drivers *drivers) : drivers(drivers) {}
    ControlOperatorInterface(const ControlOperatorInterface &) = delete;
    ControlOperatorInterface &operator=(const ControlOperatorInterface &) = delete;
    mockable ~ControlOperatorInterface() = default;

    ///< @return the value used for chassis movement forward and backward, between -1 and 1.
    mockable float getChassisXInput();

    ///< @return the value used for chassis movement side to side, between -1 and 1.
    mockable float getChassisYInput();

    ///< @return the value used for chassis rotation, between -1 and 1.
    mockable float getChassisRInput();

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    mockable float getTurretYawInput();

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    mockable float getTurretPitchInput();

    /**
     * @returns the value used for sentiel drive speed, between
     *      [-USER_STICK_SENTINEL_DRIVE_SCALAR, USER_STICK_SENTINEL_DRIVE_SCALAR].
     */
    mockable float getSentinelSpeedInput();

private:
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / 1000.0f);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / 1000.0f);

    static constexpr float USER_STICK_SENTINEL_DRIVE_SCALAR = 5000.0f;

    Drivers *drivers;

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
