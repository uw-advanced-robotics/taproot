/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef WIGGLE_DRIVE_COMMAND_HPP_
#define WIGGLE_DRIVE_COMMAND_HPP_

#include "aruwlib/algorithms/ramp.hpp"
#include "aruwlib/control/command.hpp"
#include "aruwlib/control/turret/i_turret_subsystem.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
class ChassisSubsystem;

/**
 * A command that automatically rotates the chassis back and forth, following
 * a sine wave centered around the yaw gimbal angle, while still allowing for
 * translational movement.
 */
class WiggleDriveCommand : public aruwlib::control::Command
{
public:
    WiggleDriveCommand(
        aruwlib::Drivers* drivers,
        ChassisSubsystem* chassis,
        const aruwlib::control::turret::iTurretSubsystem* turret);

    void initialize() override;

    /**
     * Updates the sine wave used for wiggling, updates the rotation PD controller,
     * and applies a rotation matrix to the <x, y> vector before passing these to
     * the chassis subsystem's `setDesiredOutput` function.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis wiggle drive"; }

private:
    struct WiggleParams
    {
        float rotationSpeed;    /// Target rotation speed (in wheel RPM)
        float turnaroundAngle;  /// Turret angle from center to change chassis rotation direction
                                /// (in degrees)
        float rotationSpeedIncrement;  /// Increment in RPM / 0.002s to change the rotation speed by
                                       /// each time step
    };

    /**
     * Use these wiggle parameters if power consumption limit is <= 45 W
     */
    static constexpr WiggleParams WIGGLE_PARAMS_45W_CUTOFF = {1700, 5, 5};
    /**
     * Use these wiggle parameters if power consumption limit is within (45, 60] W
     */
    static constexpr WiggleParams WIGGLE_PARAMS_60W_CUTOFF = {2300, 5, 10};
    /**
     * Use these wiggle parameters if power consumption limit is within (60, 80] W
     */
    static constexpr WiggleParams WIGGLE_PARAMS_80W_CUTOFF = {2500, 5, 12};
    /**
     * Use these wiggle parameters if power consumption limit is greater than 80 W
     */
    static constexpr WiggleParams WIGGLE_PARAMS_MAX_CUTOFF = {3000, 5, 15};

    static constexpr float WIGGLE_ROTATE_KP = -300.0f;
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING = 0.5f;

    aruwlib::Drivers* drivers;
    ChassisSubsystem* chassis;
    const aruwlib::control::turret::iTurretSubsystem* turret;

    aruwlib::algorithms::Ramp rotationSpeedRamp;

    int8_t rotationSign;

    const WiggleParams& getWiggleParams() const;
};  // class WiggleDriveCommand

}  // namespace chassis

}  // namespace aruwsrc

#endif  // WIGGLE_DRIVE_COMMAND_HPP_
