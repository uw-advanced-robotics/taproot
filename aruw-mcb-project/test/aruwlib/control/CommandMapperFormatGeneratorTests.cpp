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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/CommandMapperFormatGenerator.hpp>
#include <gtest/gtest.h>

#include "aruwsrc/control/robot_control.hpp"

#include "TestCommand.hpp"
#include "TestSubsystem.hpp"

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
    cm.addHoldMapping(
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
        {&tc});

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
    cm.addHoldMapping(
        RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN),
        {&tc1, &tc2});

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
    cm.addHoldMapping(RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP), {&tc});

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
    cm.addHoldMapping(RemoteMapState({Remote::Key::A, Remote::Key::B}), {&tc});

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
    cm.addHoldMapping(
        RemoteMapState({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D}),
        {&tc});

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
    cm.addHoldMapping(RemoteMapState(RemoteMapState::MouseButton::LEFT), {&tc});

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
    cm.addHoldMapping(ms, {&tc});

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[left mouse pressed, right mouse pressed]:\t[test command]", mappings[0]);
}

TEST(CommandMapperFormatGenerator, generateMappings_default_mapping_no_commands)
{
    Drivers drivers;
    CommandMapper cm(&drivers);
    CommandMapperFormatGenerator formatGenerator(cm);
    cm.addHoldMapping(RemoteMapState(), {});

    std::vector<std::string> mappings = formatGenerator.generateMappings();
    EXPECT_EQ(1, mappings.size());
    EXPECT_EQ("[none]:\t[none]", mappings[0]);
}
