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

#include <gtest/gtest.h>

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_binding.hpp"
#include "tap/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace tap::control;
using tap::Drivers;
using namespace tap::communication::serial;

TEST(CommandMapper, getSize_returns_number_of_valid_trigger_bindings_added)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);

    Trigger trigger1(&drivers, [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    });

    Trigger trigger2(&drivers, [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
    });

    Trigger trigger3(&drivers, [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    });

    EXPECT_EQ(0, cm.getSize());
    // adds trigger bindings to command mapper
    trigger1.whileTrue(&tc1);
    trigger2.whileTrue(&tc2);
    EXPECT_EQ(2, cm.getSize());
    trigger3.whileTrue(&tc3);
    EXPECT_EQ(3, cm.getSize());
}

TEST(CommandMapper, getAtIndex_nullptr_returned_if_greater_than_number_of_trigger_bindings_size)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    Trigger trigger(&drivers, [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    });
    trigger.whileTrue(&tc);

    EXPECT_NE(nullptr, cm.getAtIndex(0));
    EXPECT_EQ(nullptr, cm.getAtIndex(1));
    EXPECT_EQ(nullptr, cm.getAtIndex(2));
}

TEST(CommandMapper, getAtIndex_returns_correct_TriggerBinding_if_index_valid)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger1(&drivers, [leftSwitchDown]() { return leftSwitchDown(); });
    trigger1.whileTrue(&tc1);

    std::function<bool()> leftSwitchMid = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
    };
    Trigger trigger2(&drivers, [leftSwitchMid]() { return leftSwitchMid(); });
    trigger2.whileTrue(&tc2);

    std::function<bool()> leftSwitchUp = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    };
    Trigger trigger3(&drivers, [leftSwitchUp]() { return leftSwitchUp(); });
    trigger3.whileTrue(&tc3);

    const TriggerBinding *binding1 = cm.getAtIndex(0);
    EXPECT_EQ(
        TriggerBinding(&drivers, leftSwitchDown, &tc1, TriggerBinding::Type::WHILE_TRUE),
        *binding1);

    const TriggerBinding *binding2 = cm.getAtIndex(1);
    EXPECT_EQ(
        TriggerBinding(&drivers, leftSwitchMid, &tc2, TriggerBinding::Type::WHILE_TRUE),
        *binding2);

    const TriggerBinding *binding3 = cm.getAtIndex(2);
    EXPECT_EQ(
        TriggerBinding(&drivers, leftSwitchUp, &tc3, TriggerBinding::Type::WHILE_TRUE),
        *binding3);
}

TEST(CommandMapper, add_whileTrue_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); });
    trigger.whileTrue(&tc);

    const TriggerBinding *binding = cm.getAtIndex(0);
    TriggerBinding sameBinding(&drivers, leftSwitchDown, &tc, TriggerBinding::Type::WHILE_TRUE);
    EXPECT_NE(nullptr, binding);
    EXPECT_EQ(sameBinding, *binding);
}

TEST(CommandMapper, add_toggleOnTrue_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); });
    trigger.toggleOnTrue(&tc);

    const TriggerBinding *binding = cm.getAtIndex(0);
    TriggerBinding sameBinding(&drivers, leftSwitchDown, &tc, TriggerBinding::Type::TOGGLE_ON_TRUE);
    EXPECT_NE(nullptr, binding);
    EXPECT_EQ(sameBinding, *binding);
}

TEST(CommandMapper, add_onTrue_successfully_adds_mapping)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); });
    trigger.onTrue(&tc);

    const TriggerBinding *binding = cm.getAtIndex(0);
    TriggerBinding sameBinding(&drivers, leftSwitchDown, &tc, TriggerBinding::Type::ON_TRUE);
    EXPECT_NE(nullptr, binding);
    EXPECT_EQ(sameBinding, *binding);
}
