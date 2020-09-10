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

#ifndef __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__
#define __AGITATOR_SHOOT_COMPRISED_COMMANDS_HPP__

#include "agitator_shoot_comprised_command.hpp"

namespace aruwsrc
{
namespace agitator
{
class ShootFastComprisedCommand : public ShootComprisedCommand
{
public:
    explicit ShootFastComprisedCommand(AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              50,
              20)
    {
    }
};

class ShootSlowComprisedCommand : public ShootComprisedCommand
{
public:
    explicit ShootSlowComprisedCommand(AgitatorSubsystem* agitator17mm)
        : ShootComprisedCommand(
              agitator17mm,
              aruwlib::algorithms::PI / 5.0f,
              aruwlib::algorithms::PI / 2.0f,
              300,
              100)
    {
    }
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
