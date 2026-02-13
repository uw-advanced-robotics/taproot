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

#include "trigger.hpp"

#include <functional>
#include <memory>
#include <vector>

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "trigger_binding.hpp"

namespace tap
{
namespace control
{
Trigger::Trigger(Drivers *drivers, std::function<bool()> condition)
    : drivers(drivers),
      condition(condition)
{
}

Trigger Trigger::operator&&(const Trigger &other) const
{
    auto c1 = condition, c2 = other.condition;
    return Trigger(drivers, [c1, c2]() { return c1() && c2(); });
};

Trigger Trigger::operator||(const Trigger &other) const
{
    auto c1 = condition, c2 = other.condition;
    return Trigger(drivers, [c1, c2]() { return c1() || c2(); });
};

Trigger Trigger::operator^(const Trigger &other) const
{
    auto c1 = condition, c2 = other.condition;
    return Trigger(drivers, [c1, c2]() { return c1() ^ c2(); });
}

Trigger Trigger::operator!() const
{
    auto c1 = condition;
    return Trigger(drivers, [c1]() { return !c1(); });
}

void Trigger::onTrue(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::ON_TRUE)));
}

void Trigger::onFalse(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::ON_FALSE)));
}

void Trigger::whileTrue(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::WHILE_TRUE)));
}

void Trigger::whileFalse(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::WHILE_FALSE)));
}

void Trigger::toggleOnTrue(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::TOGGLE_ON_TRUE)));
}

void Trigger::toggleOnFalse(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::TOGGLE_ON_FALSE)));
}

void Trigger::onChange(Command *command)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::ON_CHANGE)));
}

void Trigger::debounce(Command *command, uint32_t timeout)
{
    drivers->commandMapper.addTriggerBinding(std::make_unique<TriggerBinding>(
        TriggerBinding(drivers, condition, command, TriggerBinding::Type::DEBOUNCE, timeout)));
}
}  // namespace control
}  // namespace tap