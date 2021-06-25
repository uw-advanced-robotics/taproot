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

#ifndef TEST_COMMAND_HPP_
#define TEST_COMMAND_HPP_

#include "aruwlib/control/command.hpp"

#include "TestSubsystem.hpp"

namespace aruwlib
{
namespace control
{
class TestCommand : public aruwlib::control::Command
{
public:
    TestCommand(TestSubsystem *ts) : finished(false), s(ts) { addSubsystemRequirement(s); }

    void initialize() override {}
    void execute() override {}
    void end(bool) override {}
    bool isFinished() const override { return finished; }
    const char *getName() const override { return "test command"; }
    void setFinished(bool finished) { this->finished = finished; }

private:
    bool finished;
    TestSubsystem *s;
};  // TestCommand
}  // namespace control
}  // namespace aruwlib

#endif  // TEST_COMMAND_HPP_
