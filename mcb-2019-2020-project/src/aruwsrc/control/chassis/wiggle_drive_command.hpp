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

#ifndef __WIGGLE_DRIVE_COMMAND_HPP__
#define __WIGGLE_DRIVE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
class WiggleDriveCommand : public aruwlib::control::Command
{
public:
    explicit WiggleDriveCommand(ChassisSubsystem* chassis, aruwsrc::turret::TurretSubsystem* turret)
        : chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis wiggle drive command"; }

private:
    static constexpr float WIGGLE_PERIOD = 1600.0f;
    static constexpr float WIGGLE_MAX_ROTATE_ANGLE = 60.0f;
    static constexpr float WIGGLE_ROTATE_KP = -250.0f;
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING = 0.5f;
    static constexpr float WIGGLE_OUT_OF_CENTER_MAX_ROTATE_ERR = 10.0f;

    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem* turret;

    uint32_t timeOffset = 0;
    float startTimeForAngleOffset = 0.0f;
    bool outOfCenter = false;

    // sin curve to determine angle to rotate to based on current "time"
    float wiggleSin(float time);
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
