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

#include <memory>

#include <gtest/gtest.h>

#include "tap/control/trigger.hpp"
#include "tap/control/trigger_binding.hpp"
#include "tap/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using tap::Drivers;
using namespace tap::communication::serial;

TEST(Trigger, trigger_composition_AND)
{
    Drivers drivers;
    bool a = false;
    bool b = false;
    std::function<bool()> c1 = [&a]() { return a; };
    std::function<bool()> c2 = [&b]() { return b; };

    Trigger trigger1 = Trigger(&drivers, [&a]() { return a; });
    Trigger trigger2 = Trigger(&drivers, [&b]() { return b; });
    Trigger composedTrigger = trigger1 && trigger2;

    EXPECT_FALSE(composedTrigger.get());

    b = true;
    EXPECT_FALSE(composedTrigger.get());

    a = true;
    b = false;
    EXPECT_FALSE(composedTrigger.get());

    a = true;
    b = true;
    EXPECT_TRUE(composedTrigger.get());
}

TEST(Trigger, trigger_composition_OR)
{
    Drivers drivers;
    bool a = false;
    bool b = false;
    std::function<bool()> c1 = [&a]() { return a; };
    std::function<bool()> c2 = [&b]() { return b; };

    Trigger trigger1 = Trigger(&drivers, [&a]() { return a; });
    Trigger trigger2 = Trigger(&drivers, [&b]() { return b; });
    Trigger composedTrigger = trigger1 || trigger2;

    EXPECT_FALSE(composedTrigger.get());

    b = true;
    EXPECT_TRUE(composedTrigger.get());

    a = true;
    b = false;
    EXPECT_TRUE(composedTrigger.get());

    a = true;
    b = true;
    EXPECT_TRUE(composedTrigger.get());
}

TEST(Trigger, trigger_composition_XOR)
{
    Drivers drivers;
    bool a = false;
    bool b = false;
    std::function<bool()> c1 = [&a]() { return a; };
    std::function<bool()> c2 = [&b]() { return b; };

    Trigger trigger1 = Trigger(&drivers, [&a]() { return a; });
    Trigger trigger2 = Trigger(&drivers, [&b]() { return b; });
    Trigger composedTrigger = trigger1 ^ trigger2;

    EXPECT_FALSE(composedTrigger.get());

    b = true;
    EXPECT_TRUE(composedTrigger.get());

    a = true;
    b = false;
    EXPECT_TRUE(composedTrigger.get());

    a = true;
    b = true;
    EXPECT_FALSE(composedTrigger.get());
}

TEST(Trigger, trigger_negation)
{
    Drivers drivers;
    bool a = false;

    Trigger trigger = Trigger(&drivers, [&a]() { return a; });
    Trigger negTrigger = !trigger;

    EXPECT_TRUE(negTrigger.get());

    a = true;
    EXPECT_FALSE(negTrigger.get());
}

bool isCommandInCommandMapper(const TriggerBinding &tb, const CommandMapper &cm)
{
    for (std::size_t i = 0; i < cm.getBindingSize(); i++)
    {
        if (*cm.getBindingAtIndex(i) == tb)
        {
            return true;
        }
    }
    return false;
}

TEST(Trigger, trigger_bindings_added_to_command_mapper)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    std::function<bool()> condition = [&drivers]() { return drivers.remote.getMouseL(); };

    TriggerBinding onTrueBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::ON_TRUE);
    EXPECT_FALSE(isCommandInCommandMapper(onTrueBinding, drivers.commandMapper));
    Trigger t1 = Trigger(&drivers, condition).onTrue(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(onTrueBinding, drivers.commandMapper));

    TriggerBinding onFalseBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::ON_FALSE);
    EXPECT_FALSE(isCommandInCommandMapper(onFalseBinding, drivers.commandMapper));
    Trigger t2 = Trigger(&drivers, condition).onFalse(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(onFalseBinding, drivers.commandMapper));

    TriggerBinding whileTrueBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::WHILE_TRUE);
    EXPECT_FALSE(isCommandInCommandMapper(whileTrueBinding, drivers.commandMapper));
    Trigger t3 = Trigger(&drivers, condition).whileTrue(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(whileTrueBinding, drivers.commandMapper));

    TriggerBinding whileFalseBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::WHILE_FALSE);
    EXPECT_FALSE(isCommandInCommandMapper(whileFalseBinding, drivers.commandMapper));
    Trigger t4 = Trigger(&drivers, condition).whileFalse(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(whileFalseBinding, drivers.commandMapper));

    TriggerBinding toggleOnTrueBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::TOGGLE_ON_TRUE);
    EXPECT_FALSE(isCommandInCommandMapper(toggleOnTrueBinding, drivers.commandMapper));
    Trigger t5 = Trigger(&drivers, condition).toggleOnTrue(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(toggleOnTrueBinding, drivers.commandMapper));

    TriggerBinding toggleOnFalseBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::TOGGLE_ON_FALSE);
    EXPECT_FALSE(isCommandInCommandMapper(toggleOnFalseBinding, drivers.commandMapper));
    Trigger t6 = Trigger(&drivers, condition).toggleOnFalse(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(toggleOnFalseBinding, drivers.commandMapper));

    TriggerBinding onChangeBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::ON_CHANGE);
    EXPECT_FALSE(isCommandInCommandMapper(onChangeBinding, drivers.commandMapper));
    Trigger t7 = Trigger(&drivers, condition).onChange(&tc);
    EXPECT_TRUE(isCommandInCommandMapper(onChangeBinding, drivers.commandMapper));

    TriggerBinding debounceBinding =
        TriggerBinding(&drivers, condition, &tc, TriggerBinding::Type::DEBOUNCE);
    EXPECT_FALSE(isCommandInCommandMapper(debounceBinding, drivers.commandMapper));
    Trigger t8 = Trigger(&drivers, condition).debounce(&tc, 1000);
    EXPECT_TRUE(isCommandInCommandMapper(debounceBinding, drivers.commandMapper));
}