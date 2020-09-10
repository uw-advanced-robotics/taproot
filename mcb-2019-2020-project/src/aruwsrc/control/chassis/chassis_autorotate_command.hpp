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

#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
class ChassisAutorotateCommand : public aruwlib::control::Command
{
public:
    explicit ChassisAutorotateCommand(
        ChassisSubsystem* chassis,
        aruwsrc::turret::TurretSubsystem const* turret)
        : chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis autorotate command"; }

private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem const* turret;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
