/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CHASSIS_SUBSYSTEM_INTERFACE_
#define CHASSIS_SUBSYSTEM_INTERFACE_

#include "tap/motor/dji_motor.hpp"

#include "../subsystem.hpp"

namespace tap::control::chassis
{
class ChassisSubsystemInterface : public Subsystem
{
public:
    ChassisSubsystemInterface(Drivers *drivers) : Subsystem(drivers) {}

    /**
     * @return the number of chassis motors
     */
    virtual inline int getNumChassisMotors() const = 0;

    virtual inline int16_t getLeftFrontRpmActual() const = 0;
    virtual inline int16_t getLeftBackRpmActual() const = 0;
    virtual inline int16_t getRightFrontRpmActual() const = 0;
    virtual inline int16_t getRightBackRpmActual() const = 0;

    // Return the unwrapped angular position of the wheel. Value is undefined
    // unless corresponding motor is online.
    virtual inline float getLeftFrontAngleUnwrappedActual() const = 0;
    virtual inline float getLeftBackAngleUnwrappedActual() const = 0;
    virtual inline float getRightFrontAngleUnwrappedActual() const = 0;
    virtual inline float getRightBackAngleUnwrappedActual() const = 0;

    // Return whether the specified motor is online (must have received
    // a message from the motor within some timeframe to be considered online)
    virtual inline bool isLeftFrontMotorOnline() const = 0;
    virtual inline bool isLeftBackMotorOnline() const = 0;
    virtual inline bool isRightFrontMotorOnline() const = 0;
    virtual inline bool isRightBackMotorOnline() const = 0;

    // Returns true if all four motors are
    inline bool areAllMotorsOnline()
    {
        return isLeftFrontMotorOnline() && isLeftBackMotorOnline && isRightFrontMotorOnline() &&
               isRightBackMotorOnline();
    }
};
}  // namespace tap::control::chassis

#endif  // CHASSIS_SUBSYSTEM_INTERFACE_
