#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <map>
#include <list>
#include <utility>
#include <numeric>
#include "command.hpp"
#include "command_scheduler.hpp"
#include "src/aruwlib/communication/remote.hpp"

/* 
 * Control for mapping commands to actions. For example, all the remote
 * mappings will be handled here. One creates a new RemoteMap
 * using the RemoteMapper class (i.e. press a button on the
 * keyboard) and a pointer to a command to be executed when commanded.
 */

using namespace aruwlib;

namespace aruwlib {

namespace control {

class IoMapper

{
friend class aruwlib::Remote;

 private:
    /**
     * PRESS: The command is added exactly once when you enter the state that matches
     *        the correct state.
     * 
     * HOLD: The command is added once when the state matches the correct and removed
     *       when you leave this state.
     * 
     * HOLD_REPEAT: The command is added when the state matches the correct state and
     *              is added again every time the command ends.
     * 
     * TOGGLE: The command is added when the state matches the correct state and
     *         removed when you reenter the state again.
     */
    enum MapType {
        HOLD_REPEAT,
        HOLD,
        PRESS,
        TOGGLE
    };

    struct RemoteMap {
        const Remote::SwitchState lSwitch;
        const Remote::SwitchState rSwitch;
        const uint16_t keys;

        RemoteMap(Remote::SwitchState ls, Remote::SwitchState rs, uint16_t k) :
        lSwitch(ls), rSwitch(rs), keys(k) {}
    };

    struct MapInfo {
        bool pressed = false;
        bool toggled = false;
        MapType type;
        Command* command;
        MapInfo(MapType mt, Command* sp) : type(mt), command(sp) {}
    };

    struct compareRemoteMapPtrs {
        bool operator()(const RemoteMap* a, const RemoteMap* b) const {
            return a->keys != b->keys || a->lSwitch != b->lSwitch || a->rSwitch != b->rSwitch;
        }
    };

    static std::map<RemoteMap*, MapInfo*, compareRemoteMapPtrs> remoteMappings;

    /**
     * Iterates through all the current mappings to see which buttons are pressed
     * in order to determine which commands should be added to the scheduler. Then,
     * for each button pressed/combination of buttons, executes the commands.
     */

    static void handleKeyStateChange(uint16_t key,
                                     Remote::SwitchState leftSwitch,
                                     Remote::SwitchState rightSwitch);

    static void addMap(RemoteMap* mapping, MapInfo* mapInfo);

 public:
    /**
     * Attaches a command to a remote control mapping which executes when a mapping is pressed
     * Note: a press mapping is only executed once
     */

    static void addPressMapping(RemoteMap* mapping, Command* command);

    /**
     * Attaches a command to a remote control mapping which executes while a mapping is held
     * Note: a hold mapping is interrupted when the key is no longer being held
     */

    static void addHoldMapping(RemoteMap* mapping, Command* command);

    static void addHoldRepeatMapping(RemoteMap* mapping, Command* command);

    /**
     * Attaches a command to a remote control mapping which executes whenever a mapping is toggled
     * Note: a toggle mapping is interrupted when the key is untoggled
     */

    static void addToggleMapping(RemoteMap* mapping, Command* command);

    /*
     *   Returns a RemoteMap with dependencies on specified keys and no switches
     */

    static RemoteMap* newKeyMap(std::list<Remote::Key> k);

    /*
     *   Returns a RemoteMap with dependencies on one specified switch and given keys
     */

    static RemoteMap* newKeyMap(Remote::Switch sw,
                                          Remote::SwitchState switchState,
                                          std::list<Remote::Key> k = {});

    /*
     *    Returns a RemoteMap with dependencies on both specified switches and given keys
     */

    static RemoteMap* newKeyMap(Remote::SwitchState leftSwitchState,
                                          Remote::SwitchState rightSwitchState,
                                          std::list<Remote::Key> k = {});
};

}  // namespace control

}  // namespace aruwlib

#endif
