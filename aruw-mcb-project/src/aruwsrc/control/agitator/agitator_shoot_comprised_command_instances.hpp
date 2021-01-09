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

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_

#include <aruwlib/Drivers.hpp>

#include "agitator_shoot_comprised_command.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for fast rotation speed for a soldier robot's
 * agitator.
 */
class ShootFastComprisedCommand : public ShootComprisedCommand
{
public:
    ShootFastComprisedCommand(aruwlib::Drivers* drivers, AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              drivers,
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              50,
              20)
    {
    }
};  // class ShootFastComprisedCommand

/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for slow rotation speed for a soldier robot's
 * agitator.
 */
class ShootSlowComprisedCommand : public ShootComprisedCommand
{
public:
    ShootSlowComprisedCommand(aruwlib::Drivers* drivers, AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              drivers,
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              300,
              100)
    {
    }
};  // class ShootSlowComprisedCommand

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_
