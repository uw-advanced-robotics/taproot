#include "controller_mapper.hpp"

namespace aruwlib {

std::map<IoMapper::RemoteMap*,
         IoMapper::MapInfo*,
         IoMapper::compareRemoteMapPtrs> IoMapper::remoteMappings;

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
                        control::CommandScheduler::addCommand(mi->command);
                    }
                    break;
                case HOLD:
                    if (!mi->pressed) {
                        control::CommandScheduler::addCommand(mi->command);
                        mi->pressed = true;
                    }
                    break;
                case HOLD_REPEAT:  // spam add the command
                    if (!control::CommandScheduler::isCommandScheduled(mi->command)) {
                        control::CommandScheduler::addCommand(mi->command);
                    }
                    break;
                case TOGGLE:
                    if (!mi->pressed) {
                        if (mi->toggled) {
                            control::CommandScheduler::removeCommand(mi->command, true);
                            mi->toggled = false;
                        } else {
                            control::CommandScheduler::addCommand(mi->command);
                            mi->toggled = true;
                        }
                        mi->pressed = true;
                    }
                    break;
                default:
                    break;
                }
        } else {
            if ((mi->type == HOLD && mi->pressed) || mi->type == HOLD_REPEAT) {
                control::CommandScheduler::removeCommand(mi->command, true);
            }
            mi->pressed = false;
        }
    }
}

void IoMapper::addPressMapping (RemoteMap* mapping, modm::SmartPointer command) {
    addMap(mapping, new IoMapper::MapInfo(PRESS, command));
}

void IoMapper::addHoldMapping(RemoteMap* mapping, modm::SmartPointer command) {
    addMap(mapping, new IoMapper::MapInfo(HOLD, command));
}

void IoMapper::addHoldRepeatMapping(RemoteMap* mapping, modm::SmartPointer command) {
    addMap(mapping, new IoMapper::MapInfo(HOLD_REPEAT, command));
}

void IoMapper::addToggleMapping(RemoteMap* mapping, modm::SmartPointer command) {
    addMap(mapping, new IoMapper::MapInfo(TOGGLE, command));
}

void IoMapper::addMap(RemoteMap* mapping, MapInfo* mapInfo) {
    if (remoteMappings.insert(
        std::pair<RemoteMap*, MapInfo*>
        (mapping, mapInfo)).second == false) {
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

    // throw error here eventually
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

}  // namespace aruwlib
