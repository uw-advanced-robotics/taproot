
#ifndef TAPROOT_INPUT_HELD_TIMEOUT_COMMAND_MAPPING_HPP_
#define TAPROOT_INPUT_HELD_TIMEOUT_COMMAND_MAPPING_HPP_

#include "tap/architecture/conditional_timer.hpp"
#include "command_mapping.hpp"

namespace tap
{
using namespace arch;
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping for a certain period of time.
 * The Command is not removed by the CommandMapping. Instead, the mapping
 * will be removed when the Command is finished.
 *
 * Additionally, When neg keys are being used and the mapping's neg keys
 * are a subset of the remote map state, the `Command`s are removed.
 */
class InputHeldTimeoutCommandMapping : CommandMapping
{
public:
    /**
     * Constructor must take the set of `Command`s and the RemoteMapState.
     * @param durationHeld the duration in seconds that the contained map state must be a substate of the
     *      remote mapping before executing the command.
     */
    InputHeldTimeoutCommandMapping(
        Drivers *drivers,
        const std::vector<Command *> cmds,
        const RemoteMapState &rms, float durationHeld)
        : CommandMapping(drivers, cmds, rms),
          commandScheduled(false),
          durationHeld(durationHeld),
          timer(new ConditionalTimer(durationHeld))
    {
    }

    /**
     * Default destructor.
     */
    ~HoldCommandMapping() override = default;

    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool commandScheduled;
    float durationHeld;
    tap::arch::ConditionalTimer timer;

}; //class InputHeldTimeoutCommandMapping
} //namespace control
} //namespace tap