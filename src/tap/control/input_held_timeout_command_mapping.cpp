
#include "input_held_timeout_command_mapping.hpp"

namespace tap
{
namespace control
{
void InputHeldTimeoutCommandMapping::executeCommandMapping(const RemoteMapState &currState) {
    if (this->timer.execute(mappingSubset(currState) 
        && !(mapState.getNegKeysUsed() 
        && negKeysSubset(mapState, currState))))
    {
        if (!commandScheduled) {
            commandScheduled = true;
            addCommands();
        }    
    }
    else
    {
        commandScheduled = false;
    }
}
} //namespace control
} //namespace tap