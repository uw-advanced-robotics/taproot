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

    EXPECT_CALL(drivers.commandMapper, addTriggerBindingRaw(testing::_)).Times(3);

    EXPECT_EQ(0, drivers.commandMapper.getSize());
    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger1 =
        Trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); }).whileTrue(&tc1);

    std::function<bool()> leftSwitchMid = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
    };
    Trigger trigger2 =
        Trigger(&drivers, [leftSwitchMid]() { return leftSwitchMid(); }).whileTrue(&tc2);
    EXPECT_EQ(2, drivers.commandMapper.getSize());

    std::function<bool()> leftSwitchUp = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    };
    Trigger trigger3 =
        Trigger(&drivers, [leftSwitchUp]() { return leftSwitchUp(); }).whileTrue(&tc3);
    EXPECT_EQ(3, drivers.commandMapper.getSize());
}

TEST(CommandMapper, getAtIndex_nullptr_returned_if_greater_than_number_of_trigger_bindings_size)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger =
        Trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); }).whileTrue(&tc);

    EXPECT_NE(nullptr, drivers.commandMapper.getAtIndex(0));
    EXPECT_EQ(nullptr, drivers.commandMapper.getAtIndex(1));
    EXPECT_EQ(nullptr, drivers.commandMapper.getAtIndex(2));
}

TEST(CommandMapper, getAtIndex_returns_correct_TriggerBinding_if_index_valid)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);

    std::function<bool()> leftSwitchDown = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN;
    };
    Trigger trigger1 =
        Trigger(&drivers, [leftSwitchDown]() { return leftSwitchDown(); }).whileTrue(&tc1);
    TriggerBinding tb1 =
        TriggerBinding(&drivers, leftSwitchDown, &tc1, TriggerBinding::Type::WHILE_TRUE);

    std::function<bool()> leftSwitchMid = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID;
    };
    Trigger trigger2 =
        Trigger(&drivers, [leftSwitchMid]() { return leftSwitchMid(); }).whileTrue(&tc2);
    TriggerBinding tb2 =
        TriggerBinding(&drivers, leftSwitchMid, &tc2, TriggerBinding::Type::WHILE_TRUE);

    std::function<bool()> leftSwitchUp = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    };
    Trigger trigger3 =
        Trigger(&drivers, [leftSwitchUp]() { return leftSwitchUp(); }).whileTrue(&tc3);
    TriggerBinding tb3 =
        TriggerBinding(&drivers, leftSwitchUp, &tc3, TriggerBinding::Type::WHILE_TRUE);

    const TriggerBinding *binding1 = drivers.commandMapper.getAtIndex(0);
    EXPECT_EQ(tb1, *binding1);

    const TriggerBinding *binding2 = drivers.commandMapper.getAtIndex(1);
    EXPECT_EQ(tb2, *binding2);

    const TriggerBinding *binding3 = drivers.commandMapper.getAtIndex(2);
    EXPECT_EQ(tb3, *binding3);
}