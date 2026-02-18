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

#include <functional>

#include "tap/control/command.hpp"
#include "tap/control/trigger.hpp"
#include "trigger_binding.hpp"

namespace tap
{
namespace control
{

void TriggerBinding::execute()
{
    bool curTrigState = condition();
    switch (type)
    {
        case Type::ON_TRUE:
        {
            if (curTrigState && !prevTrigState)
            {
                drivers->commandScheduler.addCommand(command);
            }
            break;
        }
        case Type::ON_FALSE:
        {
            if (!curTrigState && prevTrigState)
            {
                drivers->commandScheduler.addCommand(command);
            }
            break;
        }
        case Type::WHILE_TRUE:
        {
            if (curTrigState)
            {
                drivers->commandScheduler.addCommand(command);
            }
            else
            {
                drivers->commandScheduler.removeCommand(command, false);
            }
            break;
        }
        case Type::WHILE_FALSE:
        {
            if (!curTrigState)
            {
                drivers->commandScheduler.addCommand(command);
            }
            else
            {
                drivers->commandScheduler.removeCommand(command, false);
            }
            break;
        }
        case Type::TOGGLE_ON_TRUE:
        {
            if (curTrigState && !prevTrigState)
            {
                if (drivers->commandScheduler.isCommandScheduled(command))
                {
                    drivers->commandScheduler.removeCommand(command, false);
                }
                else
                {
                    drivers->commandScheduler.addCommand(command);
                }
            }
            break;
        }
        case Type::TOGGLE_ON_FALSE:
        {
            if (!curTrigState && prevTrigState)
            {
                if (drivers->commandScheduler.isCommandScheduled(command))
                {
                    drivers->commandScheduler.removeCommand(command, false);
                }
                else
                {
                    drivers->commandScheduler.addCommand(command);
                }
            }
            break;
        }
        case Type::ON_CHANGE:
        {
            if ((curTrigState && !prevTrigState) || (!curTrigState && prevTrigState))
            {
                drivers->commandScheduler.addCommand(command);
            }
            break;
        }
        case Type::DEBOUNCE:
        {
            if (prevTrigState && curTrigState)
            {
                curTime = tap::arch::clock::getTimeMilliseconds();
            }
            if (tap::arch::clock::getTimeMilliseconds() - curTime > debounceTimeout)
            {
                drivers->commandScheduler.addCommand(command);
            }
            break;
        }
    }
    prevTrigState = curTrigState;
}
}  // namespace control
}  // namespace tap