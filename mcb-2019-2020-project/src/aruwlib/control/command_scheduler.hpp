/**
 * Class for handling all the commands you would like to currently runs.
 * Only knows how to run commands and refresh subsystems and nothing else.
 * 
 * Contains list of all commands and subsystems that need to be run
 * currently, runs these commands and refresh the subsystems  every time
 * run is called. Uses isFinished function from command to determine if
 * a command should be completed.
 *
 * The goal of this class is for the user to interace with this as little
 * as possible. Aside from run, the user should be interacting with the
 * command class and subsystem class to add and remove commands from the
 * sehcduler.
 */

#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

#include <map>
#include <modm/container/linked_list.hpp>
#include <rm-dev-board-a/board.hpp>

#include "subsystem.hpp"

namespace aruwlib
{

namespace control
{

class CommandScheduler
{
 public:
    CommandScheduler() : subsystemToCommandMap()
    {}

    void runCommands();

    void run();

    void removeCommand(Command* command, bool interrupted);

    void registerSubsystem(Subsystem* subsystem);

    bool isSubsystemRegistered(Subsystem* subsystem) const;

    bool isCommandScheduled(Command* command) const;

    void addCommand(Command* commandToAdd);

    static CommandScheduler& getMainScheduler();

 private:
    // maximum time before we start erroring, in seconds
    static constexpr float MAX_ALLOWABLE_SCHEDULER_RUNTIME = 100;

    // a map containing keys of subsystems, pairs of Commands
    std::map<Subsystem*, Command*> subsystemToCommandMap;

    static uint32_t commandSchedulerTimestamp;

    static CommandScheduler mainScheduler;
};

}  // namespace control

}  // namespace aruwlib

#endif
