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

#ifndef __EXAMPLE_COMPRISED_COMAND_HPP__
#define __EXAMPLE_COMPRISED_COMAND_HPP__

#include "aruwlib/control/comprised_command.hpp"

#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class ExampleComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    explicit ExampleComprisedCommand(aruwlib::Drivers* drivers, ExampleSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "example comprised"; }

private:
    ExampleCommand exampleCommand;

    ExampleCommand otherExampleCommand;

    aruwlib::arch::MilliTimeout switchTimer;

    bool switchCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
