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

#include "squeeze_grabber_command.hpp"

#include <aruwlib/control/subsystem.hpp>

#include "grabber_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
SqueezeGrabberCommand::SqueezeGrabberCommand(GrabberSubsystem* subsystem)
    : Command(),
      grabber(subsystem)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(grabber));
}

void SqueezeGrabberCommand::initialize() { grabber->setSqueezed(true); }

void SqueezeGrabberCommand::execute() {}

void SqueezeGrabberCommand::end(bool) { grabber->setSqueezed(false); }

bool SqueezeGrabberCommand::isFinished() const { return false; }
}  // namespace engineer

}  // namespace aruwsrc
