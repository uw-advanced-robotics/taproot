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

#include "command_mapper.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "command_scheduler.hpp"

namespace aruwlib
{
namespace control
{
void CommandMapper::handleKeyStateChange(
    uint16_t key,
    Remote::SwitchState leftSwitch,
    Remote::SwitchState rightSwitch)
{
    for (std::pair<RemoteMap*, MapInfo*> it : remoteMappings)
    {
        CommandMapper::RemoteMap* rm = it.first;
        CommandMapper::MapInfo* mi = it.second;

        bool triggeredCommandStateChange =
            ((leftSwitch == rm->lSwitch || rm->lSwitch == Remote::SwitchState::UNKNOWN) &&
             (rightSwitch == rm->rSwitch || rm->rSwitch == Remote::SwitchState::UNKNOWN)) &&
            ((key & rm->keys) == rm->keys);

        // adding command when remote things are switched to matching position
        // remote switches and key presses are independent. running a command depends upon one
        // or another or both
        if (triggeredCommandStateChange)
        {
            switch (mi->type)
            {
                case PRESS:
                    if (!mi->pressed)
                    {
                        mi->pressed = true;
                        drivers->commandScheduler.addCommand(mi->command);
                    }
                    break;
                case HOLD:
                    if (!mi->pressed)
                    {
                        drivers->commandScheduler.addCommand(mi->command);
                        mi->pressed = true;
                    }
                    break;
                case HOLD_REPEAT:  // spam add the command
                    if (!drivers->commandScheduler.isCommandScheduled(mi->command))
                    {
                        drivers->commandScheduler.addCommand(mi->command);
                    }
                    break;
                case TOGGLE:
                    if (!mi->pressed)
                    {
                        if (mi->toggled)
                        {
                            drivers->commandScheduler.removeCommand(mi->command, true);
                            mi->toggled = false;
                        }
                        else
                        {
                            drivers->commandScheduler.addCommand(mi->command);
                            mi->toggled = true;
                        }
                        mi->pressed = true;
                    }
                    break;
                default:
                    break;
            }
        }
        else
        {
            if ((mi->type == HOLD && mi->pressed) || (mi->type == HOLD_REPEAT))
            {
                drivers->commandScheduler.removeCommand(mi->command, true);
            }
            mi->pressed = false;
        }
    }
}

void CommandMapper::addPressMapping(RemoteMap* mapping, Command* command)
{
    addMap(mapping, new CommandMapper::MapInfo(PRESS, command));
}

void CommandMapper::addHoldMapping(RemoteMap* mapping, Command* command)
{
    addMap(mapping, new CommandMapper::MapInfo(HOLD, command));
}

void CommandMapper::addHoldRepeatMapping(RemoteMap* mapping, Command* command)
{
    addMap(mapping, new CommandMapper::MapInfo(HOLD_REPEAT, command));
}

void CommandMapper::addToggleMapping(RemoteMap* mapping, Command* command)
{
    addMap(mapping, new CommandMapper::MapInfo(TOGGLE, command));
}

void CommandMapper::addMap(RemoteMap* mapping, MapInfo* mapInfo)
{
    if (remoteMappings.insert(std::pair<RemoteMap*, MapInfo*>(mapping, mapInfo)).second == false)
    {
        RAISE_ERROR(
            drivers,
            "failed to insert io mapping",
            aruwlib::errors::CONTROLLER_MAPPER,
            aruwlib::errors::INVALID_ADD);
    }
}

CommandMapper::RemoteMap* CommandMapper::newKeyMap(std::list<Remote::Key> keySet)
{
    return newKeyMap(Remote::SwitchState::UNKNOWN, Remote::SwitchState::UNKNOWN, keySet);
}

CommandMapper::RemoteMap* CommandMapper::newKeyMap(
    Remote::Switch sw,
    Remote::SwitchState switchState,
    std::list<Remote::Key> keySet)
{
    if (sw == Remote::Switch::LEFT_SWITCH)
    {
        return newKeyMap(switchState, Remote::SwitchState::UNKNOWN, keySet);
    }
    else if (sw == Remote::Switch::RIGHT_SWITCH)
    {
        return newKeyMap(Remote::SwitchState::UNKNOWN, switchState, keySet);
    }

    RAISE_ERROR(
        drivers,
        "adding a key map with unknown switch state",
        aruwlib::errors::CONTROLLER_MAPPER,
        aruwlib::errors::INVALID_KEY_MAP_TYPE);

    return nullptr;
}

CommandMapper::RemoteMap* CommandMapper::newKeyMap(
    Remote::SwitchState leftSwitchState,
    Remote::SwitchState rightSwitchState,
    std::list<Remote::Key> keySet)
{
    uint16_t keys = std::accumulate(keySet.begin(), keySet.end(), 0, [](int acc, Remote::Key key) {
        return acc |= 1 << static_cast<uint16_t>(key);
    });

    RemoteMap* ret = new CommandMapper::RemoteMap(leftSwitchState, rightSwitchState, keys);
    return ret;
}

}  // namespace control

}  // namespace aruwlib
