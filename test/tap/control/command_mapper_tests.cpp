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

    EXPECT_EQ(0, drivers.commandMapper.getBindingSize());
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
    EXPECT_EQ(2, drivers.commandMapper.getBindingSize());

    std::function<bool()> leftSwitchUp = [&drivers]() {
        return drivers.remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    };
    Trigger trigger3 =
        Trigger(&drivers, [leftSwitchUp]() { return leftSwitchUp(); }).whileTrue(&tc3);
    EXPECT_EQ(3, drivers.commandMapper.getBindingSize());
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

    EXPECT_NE(nullptr, drivers.commandMapper.getBindingAtIndex(0));
    EXPECT_EQ(nullptr, drivers.commandMapper.getBindingAtIndex(1));
    EXPECT_EQ(nullptr, drivers.commandMapper.getBindingAtIndex(2));
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

    const TriggerBinding* binding1 = drivers.commandMapper.getBindingAtIndex(0);
    EXPECT_EQ(tb1, *binding1);

    const TriggerBinding* binding2 = drivers.commandMapper.getBindingAtIndex(1);
    EXPECT_EQ(tb2, *binding2);

    const TriggerBinding* binding3 = drivers.commandMapper.getBindingAtIndex(2);
    EXPECT_EQ(tb3, *binding3);
}

/****************** Command mapping specific tests *****************/

TEST(CommandMapper, getSize_returns_number_of_valid_maps_added)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);

    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID);
    RemoteMapState ms3(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    auto hcm1 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc1}, ms1);
    auto hcm2 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc2}, ms2);
    auto hcm3 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc3}, ms3);

    EXPECT_EQ(0, cm.getCommandMappingSize());
    cm.addMap(std::move(hcm1));
    cm.addMap(std::move(hcm2));
    EXPECT_EQ(2, cm.getCommandMappingSize());
    cm.addMap(std::move(hcm3));
    EXPECT_EQ(3, cm.getCommandMappingSize());
}

TEST(CommandMapper, getAtIndex_nullptr_returned_if_greater_than_mapper_size)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    auto hcm = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc}, ms);

    cm.addMap(std::move(hcm));
    EXPECT_NE(nullptr, cm.getCommandMappingAtIndex(0));
    EXPECT_EQ(nullptr, cm.getCommandMappingAtIndex(1));
    EXPECT_EQ(nullptr, cm.getCommandMappingAtIndex(2));
}

TEST(CommandMapper, getAtIndex_returns_correct_CommandMapping_if_index_valid)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    TestCommand tc3(&ts);
    CommandMapper cm(&drivers);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID);
    RemoteMapState ms3(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    auto hcm1 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc1}, ms1);
    auto hcm2 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc2}, ms2);
    auto hcm3 = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc3}, ms3);
    cm.addMap(std::move(hcm1));
    cm.addMap(std::move(hcm2));
    cm.addMap(std::move(hcm3));

    const CommandMapping* mapping = cm.getCommandMappingAtIndex(0);
    EXPECT_EQ(ms1, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc1, mapping->getAssociatedCommands()[0]);
    mapping = cm.getCommandMappingAtIndex(1);
    EXPECT_EQ(ms2, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc2, mapping->getAssociatedCommands()[0]);
    mapping = cm.getCommandMappingAtIndex(2);
    EXPECT_EQ(ms3, mapping->getAssociatedRemoteMapState());
    EXPECT_EQ(&tc3, mapping->getAssociatedCommands()[0]);
}

TEST(CommandMapper, addHoldMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldCommandMapping expected(&drivers, {&tc}, ms);

    auto hm = std::make_unique<HoldCommandMapping>(&drivers, std::vector<Command*>{&tc}, ms);
    cm.addMap(std::move(hm));

    const auto* actual = dynamic_cast<const HoldCommandMapping*>(cm.getCommandMappingAtIndex(0));

    ASSERT_NE(nullptr, actual);
    EXPECT_EQ(expected, *actual);
}

TEST(CommandMapper, addHoldRepeatMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldRepeatCommandMapping expected(&drivers, {&tc}, ms, true);

    auto hm =
        std::make_unique<HoldRepeatCommandMapping>(&drivers, std::vector<Command*>{&tc}, ms, true);
    cm.addMap(std::move(hm));

    const auto* actual =
        dynamic_cast<const HoldRepeatCommandMapping*>(cm.getCommandMappingAtIndex(0));

    ASSERT_NE(nullptr, actual);
    EXPECT_EQ(expected, *actual);
}

TEST(CommandMapper, addToggleMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    ToggleCommandMapping expected(&drivers, {&tc}, ms);

    auto tm = std::make_unique<ToggleCommandMapping>(&drivers, std::vector<Command*>{&tc}, ms);
    cm.addMap(std::move(tm));

    const auto* actual = dynamic_cast<const ToggleCommandMapping*>(cm.getCommandMappingAtIndex(0));

    ASSERT_NE(nullptr, actual);
    EXPECT_EQ(expected, *actual);
}

TEST(CommandMapper, addPressMapping_successfully_adds_mapping_normal_case)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);

    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

    PressCommandMapping expected(&drivers, {&tc}, ms);

    auto pm = std::make_unique<PressCommandMapping>(&drivers, std::vector<Command*>{&tc}, ms);
    cm.addMap(std::move(pm));

    const auto* actual = dynamic_cast<const PressCommandMapping*>(cm.getCommandMappingAtIndex(0));

    ASSERT_NE(nullptr, actual);
    EXPECT_EQ(expected, *actual);
}