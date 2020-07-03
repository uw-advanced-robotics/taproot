#ifndef COMMAND_MAPPER_HPP_
#define COMMAND_MAPPER_HPP_

#include <list>
#include <map>
#include <numeric>
#include <utility>

#include "aruwlib/communication/remote.hpp"

#include "command.hpp"
#include "command_scheduler.hpp"

namespace aruwlib
{
namespace control
{
/**
 * Control for mapping commands to actions. For example, all the remote
 * mappings will be handled here. One creates a new RemoteMap
 * using the RemoteMapper class (i.e. press a button on the
 * keyboard) and a pointer to a Command to be executed when commanded.
 *
 * For example, given the command `coolCommand`, to map a hold mapping
 * to the left switch in the up position, we call
 * `addHoldMapping(newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), &coolCommand);`
 *
 * Currently, a single remote mapping can map to a single Command (i.e.
 * the same switch can't control the executino of multiple Commands).
 *
 * Also, key negations have not yet been implemented (there is no way
 * to for example have a Command stop executing if a certain key is pressed).
 */
class CommandMapper
{
public:
    /**
     * A struct that represents a particular remote state. This consists of
     * a left and rgith swtich state, as well as some particular key state.
     * A RemoteMap is mapped to a particular Command. When the RemoteMap
     * and Command pair is placed in the CommandMapper, the CommandMapper
     * will initiate the Command when the RemoteMap matches the remote's
     * state.
     */
    struct RemoteMap
    {
        const Remote::SwitchState lSwitch;
        const Remote::SwitchState rSwitch;
        const uint16_t keys;

        RemoteMap(Remote::SwitchState ls, Remote::SwitchState rs, uint16_t k)
            : lSwitch(ls),
              rSwitch(rs),
              keys(k)
        {
        }
    };

    CommandMapper() = default;
    CommandMapper(CommandMapper&) = delete;
    CommandMapper& operator=(CommandMapper&) = default;

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once each time the mapping is satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addPressMapping(RemoteMap* mapping, Command* Command);

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once when the mapping is satisfied and is removed
     * when the mapping is stopped being satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldMapping(RemoteMap* mapping, Command* Command);

    /**
     * Attaches a Command to a remote control mapping which is added to
     * the CommandScheduler when the remote state matches the passed in
     * mapping, starting the Command over while the mapping is still met
     * if it ever finishes.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldRepeatMapping(RemoteMap* mapping, Command* Command);

    /**
     * Attaches a Command to a remote control mapping which adds the command
     * to the CommandScheduler whenever a mapping is toggled.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     * @note a toggle mapping is interrupted when the key is untoggled.
     */
    void addToggleMapping(RemoteMap* mapping, Command* Command);

    /**
     * @return a RemoteMap with dependencies on specified keys and no switches.
     */
    static RemoteMap* newKeyMap(std::list<Remote::Key> k);

    /**
     * @return a RemoteMap with dependencies on one specified switch and given keys.
     */
    static RemoteMap* newKeyMap(
        Remote::Switch sw,
        Remote::SwitchState switchState,
        std::list<Remote::Key> k = {});

    /**
     * @return a RemoteMap with dependencies on both specified switches and given keys.
     */
    static RemoteMap* newKeyMap(
        Remote::SwitchState leftSwitchState,
        Remote::SwitchState rightSwitchState,
        std::list<Remote::Key> k = {});

private:
    friend class aruwlib::Remote;

    /**
     * - PRESS: The Command is added exactly once when you enter the state that matches
     *      the correct state.
     * - HOLD: The Command is added once when the state matches the correct and removed
     *      when you leave this state.
     * - HOLD_REPEAT: The Command is added when the state matches the correct state and
     *      is added again every time the Command ends.
     * - TOGGLE: The Command is added when the state matches the correct state and
     *      removed when you reenter the state again.
     */
    enum MapType
    {
        HOLD_REPEAT,
        HOLD,
        PRESS,
        TOGGLE
    };

    struct MapInfo
    {
        bool pressed = false;
        bool toggled = false;
        MapType type;
        Command* command;
        MapInfo(MapType mt, Command* sp) : type(mt), command(sp) {}
    };

    struct compareRemoteMapPtrs
    {
        bool operator()(const RemoteMap* a, const RemoteMap* b) const
        {
            return a->keys != b->keys || a->lSwitch != b->lSwitch || a->rSwitch != b->rSwitch;
        }
    };

    std::map<RemoteMap*, MapInfo*, compareRemoteMapPtrs> remoteMappings;

    /**
     * Iterates through all the current mappings to see which buttons are pressed
     * in order to determine which commands should be added to the scheduler. Then,
     * for each button pressed/combination of buttons, executes the commands.
     */
    void handleKeyStateChange(
        uint16_t key,
        Remote::SwitchState leftSwitch,
        Remote::SwitchState rightSwitch);

    void addMap(RemoteMap* mapping, MapInfo* mapInfo);
};  // class CommandMapper

}  // namespace control

}  // namespace aruwlib

#endif  // COMMAND_MAPPER_HPP_
