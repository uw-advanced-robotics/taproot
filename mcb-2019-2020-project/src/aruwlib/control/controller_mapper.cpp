#include "controller_mapper.hpp"
#include "aruwlib/errors/create_errors.hpp"
#include "aruwlib/Drivers.hpp"
#include "command_scheduler.hpp"

namespace aruwlib {

namespace control {

void IoMapper::handleKeyStateChange(uint16_t key,
                                    Remote::SwitchState leftSwitch,
                                    Remote::SwitchState rightSwitch) {
    for (std::pair<RemoteMap*, MapInfo*> it : remoteMappings) {
        IoMapper::RemoteMap* rm = it.first;
        IoMapper::MapInfo* mi = it.second;

        bool triggeredCommandStateChange =
            ((leftSwitch == rm->lSwitch || rm->lSwitch == Remote::SwitchState::UNKNOWN)
            && (rightSwitch == rm->rSwitch || rm->rSwitch == Remote::SwitchState::UNKNOWN))
            && ((key & rm->keys) == rm->keys);

        // adding command when remote things are switched to matching position
        // remote switches and key presses are independent. running a command depends upon one
        // or another or both
        if (triggeredCommandStateChange) {
                switch (mi->type)
                {
                case PRESS:
                    if (!mi->pressed) {
                        mi->pressed = true;
                        Drivers::commandScheduler.addCommand(mi->command);
                    }
                    break;
                case HOLD:
                    if (!mi->pressed) {
                        Drivers::commandScheduler.addCommand(mi->command);
                        mi->pressed = true;
                    }
                    break;
                case HOLD_REPEAT:  // spam add the command
                    if (!Drivers::commandScheduler.isCommandScheduled(mi->command)) {
                        Drivers::commandScheduler.addCommand(mi->command);
                    }
                    break;
                case TOGGLE:
                    if (!mi->pressed) {
                        if (mi->toggled) {
                            Drivers::commandScheduler.removeCommand(
                                mi->command, true);
                            mi->toggled = false;
                        } else {
                            Drivers::commandScheduler.addCommand(mi->command);
                            mi->toggled = true;
                        }
                        mi->pressed = true;
                    }
                    break;
                default:
                    break;
                }
        } else {
            if ((mi->type == HOLD && mi->pressed) || (mi->type == HOLD_REPEAT)) {
                Drivers::commandScheduler.removeCommand(mi->command, true);
            }
            mi->pressed = false;
        }
    }
}

void IoMapper::addPressMapping (RemoteMap* mapping, Command* command) {
    addMap(mapping, new IoMapper::MapInfo(PRESS, command));
}

void IoMapper::addHoldMapping(RemoteMap* mapping, Command* command) {
    addMap(mapping, new IoMapper::MapInfo(HOLD, command));
}

void IoMapper::addHoldRepeatMapping(RemoteMap* mapping, Command* command) {
    addMap(mapping, new IoMapper::MapInfo(HOLD_REPEAT, command));
}

void IoMapper::addToggleMapping(RemoteMap* mapping, Command* command) {
    addMap(mapping, new IoMapper::MapInfo(TOGGLE, command));
}

void IoMapper::addMap(RemoteMap* mapping, MapInfo* mapInfo) {
    if (remoteMappings.insert(
        std::pair<RemoteMap*, MapInfo*>
        (mapping, mapInfo)).second == false) {
        RAISE_ERROR("failed to insert io mapping", aruwlib::errors::CONTROLLER_MAPPER,
                aruwlib::errors::INVALID_ADD)
        // throw exception here?
    }
}

IoMapper::RemoteMap* IoMapper::newKeyMap(std::list<Remote::Key> keySet) {
    return newKeyMap(Remote::SwitchState::UNKNOWN, Remote::SwitchState::UNKNOWN, keySet);
}

IoMapper::RemoteMap* IoMapper::newKeyMap(Remote::Switch sw,
                                         Remote::SwitchState switchState,
                                         std::list<Remote::Key> keySet) {
    if (sw == Remote::Switch::LEFT_SWITCH) {
        return newKeyMap(switchState, Remote::SwitchState::UNKNOWN, keySet);
    } else if (sw == Remote::Switch::RIGHT_SWITCH) {
        return newKeyMap(Remote::SwitchState::UNKNOWN, switchState, keySet);
    }

    RAISE_ERROR("adding a key map with unknown switch state",
            aruwlib::errors::CONTROLLER_MAPPER, aruwlib::errors::INVALID_KEY_MAP_TYPE)

    return NULL;
}

IoMapper::RemoteMap* IoMapper::newKeyMap(Remote::SwitchState leftSwitchState,
                                         Remote::SwitchState rightSwitchState,
                                         std::list<Remote::Key> keySet) {
    uint16_t keys = std::accumulate(keySet.begin(), keySet.end(), 0,
    [](int acc, Remote::Key key) { return acc |= 1 << static_cast<uint16_t>(key); });

    RemoteMap* ret = new IoMapper::RemoteMap(leftSwitchState, rightSwitchState, keys);
    return ret;
}

}  // namespace control

}  // namespace aruwlib
