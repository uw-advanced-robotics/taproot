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
#include "tap/control/command_mapper_format_generator.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "test_command.hpp"
#include "test_subsystem.hpp"

using namespace aruwlib::control;
using aruwlib::Drivers;
using aruwlib::Remote;

TEST(CommandMapperFormatGenerator, generateMappings_generates_nothing_if_no_mappings)
{
    Drivers drivers;
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(0, mappings.size());
}

TEST(CommandMapperFormatGenerator, generateMappings_single_switch_mapping_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(
        &drivers,
        {&tc},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left switch: down]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_single_switch_mapping_with_multiple_commands)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc1(&ts);
    TestCommand tc2(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(
        &drivers,
        {&tc1, &tc2},
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left switch: down]:\t[test command, test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_two_switch_mapping_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(
        &drivers,
        {&tc},
        RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left switch: down, right switch: up]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_multiple_keys_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(&drivers, {&tc}, RemoteMapState({Remote::Key::A, Remote::Key::B}));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[keys: {A, B}]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_multiple_keys_and_neg_keys_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(
        &drivers,
        {&tc},
        RemoteMapState({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D}));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[keys: {A, B}, neg keys: {C, D}]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_left_mouse_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(&drivers, {&tc}, RemoteMapState(RemoteMapState::MouseButton::LEFT));
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left mouse pressed]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_left_and_right_mouse_with_single_command)
{
    Drivers drivers;
    TestSubsystem ts(&drivers);
    TestCommand tc(&ts);
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    RemoteMapState ms;
    ms.initLMouseButton();
    ms.initRMouseButton();
    HoldCommandMapping hcm(&drivers, {&tc}, ms);
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left mouse pressed, right mouse pressed]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_default_mapping_no_commands)
{
    Drivers drivers;
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    HoldCommandMapping hcm(&drivers, {}, RemoteMapState());
    cm.addMap(&hcm);

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[none]:\t[none]", mappings[0]);
}
