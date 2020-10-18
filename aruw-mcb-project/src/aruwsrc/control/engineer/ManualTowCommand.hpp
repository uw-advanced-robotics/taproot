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

#ifndef MANUAL_TOW_COMMAND_HPP_
#define MANUAL_TOW_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

#include "TowSubsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * When this command is added, the tower mechanism's clamps are triggered until the command
 * is removed.
 */
class ManualTowCommand : public aruwlib::control::Command
{
public:
    explicit ManualTowCommand(TowSubsystem* subsystem);

    void initialize() override;

    void execute() override {}

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "manual tow command"; }

private:
    TowSubsystem* towSubsystem;
};  // class ManualTowCommand
}  // namespace engineer
}  // namespace aruwsrc

#endif  // end of ManualTowCommand.hpp
