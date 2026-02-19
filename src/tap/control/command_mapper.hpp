/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_COMMAND_MAPPER_HPP_
#define TAPROOT_COMMAND_MAPPER_HPP_

#include <memory>
#include <vector>

#include "tap/communication/serial/remote.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
namespace control
{
class TriggerBinding;

class CommandMapper
{
public:
    DISALLOW_COPY_AND_ASSIGN(CommandMapper)
    mockable ~CommandMapper();
    explicit CommandMapper(Drivers*);

    mockable void pollTriggerBindings();

    mockable void addTriggerBinding(std::unique_ptr<TriggerBinding> binding);

    /**
     * @return the number of trigger bindings in the mapper.
     */
    mockable std::size_t getSize() const { return triggerBindings.size(); }

private:
    std::vector<std::unique_ptr<TriggerBinding>> triggerBindings;
};  // class CommandMapper

}  // namespace control
}  // namespace tap

#endif  // TAPROOT_COMMAND_MAPPER_HPP_
