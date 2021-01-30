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

/**
 * This code is part of aruw's repository
 *
 * Example code for a default command for the subsystem-example subsystem.
 */

#ifndef __COMMAND_EXAMPLE_HPP__
#define __COMMAND_EXAMPLE_HPP__

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>

namespace aruwsrc
{
namespace control
{
class ExampleSubsystem;

class ExampleCommand : public aruwlib::control::Command
{
public:
    ExampleCommand(ExampleSubsystem* subsystem, int speed);

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize() override;

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute() override;

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    void end(bool interrupted) override;

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const override;

    const char* getName() const override { return "example"; }

    static const int16_t DEFAULT_WHEEL_RPM = 6000;

private:
    ExampleSubsystem* subsystemExample;

    int speed;
};

}  // namespace control

}  // namespace aruwsrc
#endif
