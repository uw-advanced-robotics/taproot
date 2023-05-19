
#include "input_held_timeout_command_mapping.hpp"

#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
void InputHeldTimeoutCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (this->timer.execute(
            mappingSubset(currState) &&
            !(mapState.getNegKeysUsed() && negKeysSubset(mapState, currState))))
    {
        for (std::size_t i = 0; i < mappedCommands.size(); i++)
        {
            Command *cmd = mappedCommands[i];
            if (!drivers->commandScheduler.isCommandScheduled(cmd))
            {
                addCommands();
            }
        }
    }
}
}  // namespace control
}  // namespace tap
