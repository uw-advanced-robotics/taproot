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

#ifndef CHASSIS_DRIVE_COMMAND_MOCK_HPP_
#define CHASSIS_DRIVE_COMMAND_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/chassis/chassis_drive_command.hpp"

namespace aruwsrc
{
namespace mock
{
class ChassisDriveCommandMock : public chassis::ChassisDriveCommand
{
public:
    ChassisDriveCommandMock(aruwlib::Drivers *d, chassis::ChassisSubsystem *cs)
        : chassis::ChassisDriveCommand(d, cs)
    {
    }
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, execute, (), (override));
    MOCK_METHOD(void, end, (bool interrupted), (override));
    MOCK_METHOD(bool, isFinished, (), (const override));
    MOCK_METHOD(const char *, getName, (), (const override));
};
}  // namespace mock
}  // namespace aruwsrc

#endif  // CHASSIS_DRIVE_COMMAND_MOCK_HPP_
