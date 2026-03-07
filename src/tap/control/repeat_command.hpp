/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_REPEAT_COMMAND_HPP_
#define TAPROOT_REPEAT_COMMAND_HPP_

#include "command.hpp"

namespace tap
{
namespace control
{
class RepeatCommand : public Command {
public:
    RepeatCommand(Command *command): Command(), command(command) {}

    bool isReady() override { return command->isReady(); }

    void initialize() override {
        command->initialize();
    }

    void execute() override {
        if (ended) {
            command->initialize();
            ended = false;
        }
        command->execute();
        if (command->isFinished()) {
            command->end(false);
            ended = true;
        }
    }

    void end(bool interrupted) override {
        if (!ended) {
            command->end(interrupted);
            ended = true;
        }
    }

    bool isFinished() const override { return false; }
    
    const char* getName() const override { "repeat command"; }

private:
    Command *command;
    bool ended = false;
};
}
}

#endif