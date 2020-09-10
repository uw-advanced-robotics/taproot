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

#include "command.hpp"

#include "subsystem.hpp"

using namespace std;

namespace aruwlib
{
namespace control
{
bool Command::hasRequirement(Subsystem* requirement) const
{
    if (requirement == nullptr)
    {
        return false;
    }
    return commandRequirements.find(requirement) != commandRequirements.end();
}

void Command::addSubsystemRequirement(Subsystem* requirement)
{
    if (requirement == nullptr)
    {
        return;
    }
    // Ensure the requirement you are trying to add is not already a
    // command requirement.
    if (requirement != nullptr &&
        commandRequirements.find(requirement) == commandRequirements.end())
    {
        commandRequirements.insert(requirement);
    }
}

const set<Subsystem*>& Command::getRequirements() const { return commandRequirements; }

}  // namespace control

}  // namespace aruwlib
