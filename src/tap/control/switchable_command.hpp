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

#ifndef TAPROOT_SWITCHABLE_COMMAND_HPP_
#define TAPROOT_SWITCHABLE_COMMAND_HPP_

#include <array>
#include <functional>

#include "tap/drivers.hpp"

#include "command.hpp"
#include "repeat_command.hpp"

namespace tap
{
namespace control
{
/**
 * A class for a command that switches switch command is scheduled based on a boolean condition.
 * @param onTrue A nonterminating command to be scheduled when boolean condition is true
 * @param onFalse A nonterminating command to be scheduled when boolean condition is false
 * @param selector Whether to schedule onTrue or onFalse commands
 */
class SwitchableCommand : public Command
{
public:
    SwitchableCommand(
        Command* onTrue,
        Command* onFalse,
        std::function<bool()> selector)
        : Command(),
        onTrue(onTrue),
        onFalse(onFalse),
        selector(selector),
        lastCommandIsTrue(selector()) {}

    bool isReady() override { selector() ? onTrue->isReady() : onFalse->isReady(); }

    void initialize() override {
        selector() ? onTrue->initialize() : onFalse->initialize();
    }

    void execute() override {
        if (selector()) {
            if (!lastCommandIsTrue) {
                onTrue->initialize();
            }
            onTrue->execute();
        } else {
            if (lastCommandIsTrue) {
                onFalse->initialize();
            }
            onFalse->execute();
        }
    }

    void end(bool interrupted) override {
        onTrue->end(interrupted);
        onFalse->end(interrupted);
    }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "switchable command"; }

private:
    Command* onTrue;
    Command* onFalse;
    std::function<bool()> selector;
    bool lastCommandIsTrue;
};
}  // namespace control
}  // namespace tap

#endif