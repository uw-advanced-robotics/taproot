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

#ifndef EXTEND_XAXIS_COMMAND_HPP_
#define EXTEND_XAXIS_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace engineer
{
class XAxisSubsystem;

/**
 * Call this command to extend the x axis on the engineer.
 * This sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */
class ExtendXAxisCommand : public aruwlib::control::Command
{
public:
    explicit ExtendXAxisCommand(XAxisSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "extend x-axis command"; }

private:
    XAxisSubsystem* xAxisSubsystem;
};  // class ExtendXAxisCommand

}  // namespace engineer

}  // namespace aruwsrc

#endif  // EXTEND_XAXIS_COMMAND_HPP_
