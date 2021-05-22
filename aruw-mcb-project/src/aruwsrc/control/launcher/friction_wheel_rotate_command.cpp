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

#include "friction_wheel_rotate_command.hpp"

#include "friction_wheel_subsystem.hpp"

using aruwlib::control::Subsystem;

namespace aruwsrc
{
namespace launcher
{
FrictionWheelRotateCommand::FrictionWheelRotateCommand(FrictionWheelSubsystem* subsystem, int speed)
    : frictionWheelSubsystem(subsystem),
      speed(speed)
{
    addSubsystemRequirement(subsystem);
}

void FrictionWheelRotateCommand::initialize() {}

void FrictionWheelRotateCommand::execute() { frictionWheelSubsystem->setDesiredRpm(speed); }

void FrictionWheelRotateCommand::end(bool) { frictionWheelSubsystem->setDesiredRpm(0); }

bool FrictionWheelRotateCommand::isFinished() const { return false; }
}  // namespace launcher

}  // namespace aruwsrc
