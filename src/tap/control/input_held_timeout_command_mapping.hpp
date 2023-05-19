/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

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
     * @param durationHeld the duration in milliSeconds that the contained map state must be a substate of the
     *      remote mapping before executing the command.
     */
    InputHeldTimeoutCommandMapping(
        Drivers *drivers,
        const std::vector<Command *> cmds,
        const RemoteMapState &rms, float durationHeldms)
        : CommandMapping(drivers, cmds, rms),
        drivers(drivers),
          durationHeld(durationHeld),
          timer(durationHeld)
    {
    }

    /**
     * Default destructor.
     */
    ~InputHeldTimeoutCommandMapping() override = default;

    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    Drivers* drivers;
    float durationHeld;
    tap::arch::ConditionalMilliTimer timer;

}; //class InputHeldTimeoutCommandMapping
} //namespace control
} //namespace tap

#endif
