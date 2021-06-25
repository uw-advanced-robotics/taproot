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

#ifndef __BLINK_LED_COMMAND_HPP__
#define __BLINK_LED_COMMAND_HPP__

#include "aruwlib/Drivers.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/command.hpp"

#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class BlinkLEDCommand : public aruwlib::control::Command
{
public:
    BlinkLEDCommand(aruwlib::Drivers* drivers, aruwsrc::control::ExampleSubsystem* subsystem);

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
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const override;

    const char* getName() const override { return "blink led"; }

    aruwlib::Drivers* drivers;

    aruwlib::arch::MilliTimeout completedTimer;

    int refershCounter = 0;
    int startCounter = 0;
};

}  // namespace control

}  // namespace aruwsrc

#endif
