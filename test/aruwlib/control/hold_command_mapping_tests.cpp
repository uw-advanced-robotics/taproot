/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "aruwlib/control/hold_command_mapping.hpp"
#include "aruwlib/control/remote_map_state.hpp"
#include "aruwlib/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace aruwlib::control;
using aruwlib::Drivers;
using aruwlib::Remote;

// Adding command with switch state RemoteMapState (RMS)

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_added_if_switch_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2;
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldCommandMapping, executeCommandMapping_single_command_added_if_switch_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initRSwitch(Remote::SwitchState::UP);
    ms2.initLMouseButton();
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

// Adding commands, key based (including neg keys) RMS

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_is_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2({Remote::Key::C, Remote::Key::D});
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2 = ms1;
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldCommandMapping, executeCommandMapping_single_command_added_if_key_based_RMS_is_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C});
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);

    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_added_if_key_based_RMS_with_neg_keys_contains_matching_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C, Remote::Key::D});
    ms2.initLMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
}

// Removing already-added command from scheduler, switches and keys (including neg keys)

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_removed_if_switch_based_RMS_no_longer_equal)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState();
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_removed_if_switch_based_RMS_still_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2 = ms1;
    ms2.initKeys(42);
    ms2.initLMouseButton();
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_removed_if_key_based_RMS_still_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
    ms2 = ms1;
    commandMapping.executeCommandMapping(ms2);
}

TEST(HoldCommandMapping, executeCommandMapping_single_command_removed_if_key_based_RMS_not_subset)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::C}, {});
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::F});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_not_removed_if_key_based_RMS_does_not_matche_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(0);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::F});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_single_command_removed_if_key_based_RMS_matches_neg_keys)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    RemoteMapState ms1({Remote::Key::A, Remote::Key::B}, {Remote::Key::C});
    RemoteMapState ms2({Remote::Key::A, Remote::Key::B, Remote::Key::E});
    HoldCommandMapping commandMapping(&drivers, {&tc}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand(&tc)).Times(1);
    EXPECT_CALL(drivers.commandScheduler, removeCommand(&tc, false)).Times(1);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState({Remote::Key::A, Remote::Key::B, Remote::Key::C});
    commandMapping.executeCommandMapping(ms2);
}

TEST(
    HoldCommandMapping,
    executeCommandMapping_multiple_commands_added_and_removed_if_RMS_matches_then_does_not_match)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    RemoteMapState ms1(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    RemoteMapState ms2(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
    HoldCommandMapping commandMapping(&drivers, {&tc1, &tc2}, ms1);
    EXPECT_CALL(drivers.commandScheduler, addCommand).Times(2);
    EXPECT_CALL(drivers.commandScheduler, removeCommand).Times(2);

    commandMapping.executeCommandMapping(ms2);
    ms2 = RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    commandMapping.executeCommandMapping(ms2);
}
