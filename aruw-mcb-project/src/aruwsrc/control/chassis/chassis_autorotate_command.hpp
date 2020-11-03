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

#ifndef CHASSIS_AUTOROTATE_COMMAND_HPP_
#define CHASSIS_AUTOROTATE_COMMAND_HPP_

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * A command that continuously attempts to rotate the chasis so that the turret is
 * aligned with the center of the chassis.
 */
class ChassisAutorotateCommand : public aruwlib::control::Command
{
public:
    ChassisAutorotateCommand(
        aruwlib::Drivers* drivers,
        ChassisSubsystem* chassis,
        aruwsrc::turret::TurretSubsystem const* turret)
        : drivers(drivers),
          chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    void initialize() override;

    /**
     * Uses a PD controller to calculate the desired chassis rotation RPM based on the
     * difference between the turret angle and the center of the chassis, then
     * applies the desired rotation along with user desired <x, y> components to the
     * chassis subsystem's `setDesiredRpm` function.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis autorotate command"; }

private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    aruwlib::Drivers* drivers;
    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem const* turret;
};  // class ChassisAutorotateCommand

}  // namespace chassis

}  // namespace aruwsrc

#endif  // CHASSIS_AUTOROTATE_COMMAND_HPP_
