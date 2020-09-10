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

#include "agitator_calibrate_command.hpp"

#include <aruwlib/control/subsystem.hpp>

namespace aruwsrc
{
namespace agitator
{
AgitatorCalibrateCommand::AgitatorCalibrateCommand(AgitatorSubsystem* agitator) : agitator(agitator)
{
    this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
}

void AgitatorCalibrateCommand::initialize() { agitator->agitatorCalibrateHere(); }

void AgitatorCalibrateCommand::execute() { agitator->agitatorCalibrateHere(); }

void AgitatorCalibrateCommand::end(bool) {}

bool AgitatorCalibrateCommand::isFinished() const { return agitator->isAgitatorCalibrated(); }
}  // namespace agitator

}  // namespace aruwsrc
