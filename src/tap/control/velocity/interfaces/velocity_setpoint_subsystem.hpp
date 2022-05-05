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

#ifndef TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_HPP_
#define TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace tap
{
// Forward declaration
class Drivers;
}  // namespace tap

namespace tap::control::velocity
{
/**
 * An abstract class (usable as an interface) describing the functionalities of a subsystem which
 * uses a velocity controller to rotate something. By convention, the positional and velocity units
 * are user defined but must be consistent (i.e. if position is radians, velocity should be
 * radians/second). Positional units will be referred to as "units" in this subsystem and associated
 * commands. All time is specified to be in seconds or milliseconds.
 */
class VelocitySetpointSubsystem : public virtual tap::control::Subsystem
{
public:
    /**
     * @return the subsystem's velocity setpoint: the desired velocity of whatever is being
     * controlled, in units/second
     */
    virtual inline float getVelocitySetpoint() const = 0;

    /**
     * Sets the velocity setpoint.
     *
     * @param[in] setpoint Value in units/second, the desired velocity setpoint
     */
    virtual void setVelocitySetpoint(float setpoint) = 0;

    /**
     * @return the measured velocity of the velocity setpoint subsystem, in units/second.
     */
    virtual inline float getVelocity() = 0;

    /**
     * @return The current position of the controlled variable, in units.
     */
    virtual float getPosition() const = 0;

    /**
     * Attempts to calibrate the subsystem at the current position, such that `getPosition` will
     * return 0 units at the current position of the subsystem.
     *
     * @return `true` if the subsystem has been successfully calibrated, `false` otherwise.
     */
    virtual bool calibrateHere() = 0;

    /**
     * @return `true` if the subsystem has detected a jam.
     */
    virtual bool isJammed() = 0;

    /**
     * Call to clear the jam flag of the subsystem, indicating that the jam has been resolved.
     */
    virtual void clearJam() = 0;

    /**
     * @return `true` if the subsystem has been calibrated.
     */
    virtual inline bool isCalibrated() = 0;

    /**
     * @return `true` if the subsystem is online (i.e.: is connected).
     */
    virtual inline bool isOnline() = 0;
};

}  // namespace tap::control::velocity

#endif  // TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_HPP_
