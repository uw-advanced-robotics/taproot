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

#ifndef BEYBLADE_COMMAND_MOCK_HPP_
#define BEYBLADE_COMMAND_MOCK_HPP_

#include <aruwlib/Drivers.hpp>
#include <gmock/gmock.h>

#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class BeybladeCommandMock : public aruwsrc::chassis::BeybladeCommand
{
public:
    BeybladeCommandMock(aruwlib::Drivers *drivers) : BeybladeCommand(drivers) {}
    MOCK_METHOD(void, initialize, (), ());
    MOCK_METHOD(void, execute, (), ());
};  // class BeybladeCommandMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // BEYBLADE_COMMAND_MOCK_HPP_