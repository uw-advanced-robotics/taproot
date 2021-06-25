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

#ifndef AUTO_TOW_COMMAND_HPP_
#define AUTO_TOW_COMMAND_HPP_

#include "aruwlib/control/command.hpp"

#include "tow_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * Tow command that automatically clamps the left and right tow clamps independently
 * the first time the tow subsystem's associated limit switches have been triggered.
 * The clamps are clamped until the command is ended.
 */
class AutoTowCommand : public aruwlib::control::Command
{
public:
    explicit AutoTowCommand(TowSubsystem* subsystem);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "auto tow"; }

private:
    TowSubsystem* towSubsystem;
};  // class AutoTowCommand
}  // namespace engineer
}  // namespace aruwsrc

#endif  // AUTO_TOW_COMMAND_HPP_
