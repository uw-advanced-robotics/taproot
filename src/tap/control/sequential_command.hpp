/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_SEQUENTIAL_COMMAND_HPP_
#define TAPROOT_SEQUENTIAL_COMMAND_HPP_

#include <vector>

#include "command_scheduler_types.hpp"

namespace tap
{
namespace control
{
class Command;

/**
 * A generic extendable class for implementing a command. Each
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the virtual functions
 * in this class. See example_command.hpp for example of this.
 */
class SequentialCommand : public Command
{
public:
    SequentialCommand(const std::vector<Command*>& commands);
    const char* getName() const override;
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    std::vector<Command*> commands;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
    command_scheduler_bitmap_t allCommands;
};

}  // namespace control
}  // namespace tap

#endif  // TAPROOT_SEQUENTIAL_COMMAND_HPP_
