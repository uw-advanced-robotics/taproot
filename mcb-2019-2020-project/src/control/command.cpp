#include "src/control/command.hpp"
#include "src/control/command_scheduler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    bool Command::hasRequirement(Subsystem* requirement) const
    {
        return commandRequirements.find(requirement) != commandRequirements.end();
    }

    void Command::addSubsystemRequirement(Subsystem* requirement)
    {
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        if (requirement != nullptr &&
            commandRequirements.find(requirement) == commandRequirements.end()
        ) {
            commandRequirements.insert(requirement);
        }
    }

    const set<Subsystem*>* Command::getRequirements()
    {
        return &commandRequirements;
    }
}  // namespace control

}  // namespace aruwlib
