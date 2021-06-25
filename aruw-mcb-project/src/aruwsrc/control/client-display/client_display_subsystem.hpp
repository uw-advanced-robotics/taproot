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

#ifndef CLIENT_DISPLAY_HPP_
#define CLIENT_DISPLAY_HPP_

#include "aruwlib/control/command.hpp"
#include "aruwlib/control/subsystem.hpp"

namespace aruwsrc::display
{
/**
 * A placeholder subsystem for running the client display command
 */
class ClientDisplaySubsystem : public aruwlib::control::Subsystem
{
public:
    ClientDisplaySubsystem(aruwlib::Drivers* drivers) : Subsystem(drivers) {}
    virtual ~ClientDisplaySubsystem() {}
    const char* getName() override { return "client display"; }
};
}  // namespace aruwsrc::display

#endif  // CLIENT_DISPLAY_HPP_
