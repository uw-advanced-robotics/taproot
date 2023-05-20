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

#include "tap/communication/serial/remote.hpp"
#include "tap/control/remote_map_state.hpp"

using namespace tap::communication::serial;
using tap::control::RemoteMapState;

TEST(RemoteMapState, default_constructor_default_remote_state)
{
    RemoteMapState ms;

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, single_switchstate_constructor_only_switchstate_initialized)
{
    RemoteMapState ms(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, two_switchstate_constructor_only_switchstates_initialized)
{
    RemoteMapState ms(Remote::SwitchState::DOWN, Remote::SwitchState::UP);

    EXPECT_EQ(0, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UP, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, keyset_constructor_only_keyset_initialized)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B});

    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, keyset_negkeyset_constructor_only_keyset_negkeyset_initialized)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B}, {Remote::Key::C, Remote::Key::D});

    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());
    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::C)) | (1 << static_cast<int>(Remote::Key::D)),
        ms.getNegKeys());
        EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(true, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, equal_keyset_negkeyset_in_constructor_fails)
{
    RemoteMapState ms({Remote::Key::W}, {Remote::Key::W});

    EXPECT_EQ(1, ms.getKeys());
    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, intersecting_keyset_negkeyset_in_constructor_fails)
{
    RemoteMapState ms({Remote::Key::A, Remote::Key::B}, {Remote::Key::B, Remote::Key::C});
    EXPECT_EQ(
        (1 << static_cast<int>(Remote::Key::A)) | (1 << static_cast<int>(Remote::Key::B)),
        ms.getKeys());

    EXPECT_EQ(0, ms.getNegKeys());
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::WHEEL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::RIGHT_HORIZONTAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_VERTICAL), 1E-5);
    EXPECT_NEAR(0, ms.getChannel(Remote::Channel::LEFT_HORIZONTAL), 1E-5);
    EXPECT_EQ(false, ms.getNegKeysUsed());
    EXPECT_EQ(false, ms.getRMouseButton());
    EXPECT_EQ(false, ms.getLMouseButton());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getRSwitch());
    EXPECT_EQ(Remote::SwitchState::UNKNOWN, ms.getLSwitch());
}

TEST(RemoteMapState, initLSwitch_sets_left_SwitchState)
{
    RemoteMapState ms;
    ms.initLSwitch(Remote::SwitchState::DOWN);

    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getLSwitch());
}

TEST(RemoteMapState, initRSwitch_sets_right_SwitchState)
{
    RemoteMapState ms;
    ms.initRSwitch(Remote::SwitchState::DOWN);

    EXPECT_EQ(Remote::SwitchState::DOWN, ms.getRSwitch());
}

TEST(RemoteMapState, initKeys_sets_keyset)
{
    RemoteMapState ms;
    ms.initKeys(42);

    EXPECT_EQ(42, ms.getKeys());
}

TEST(RemoteMapState, initNegkeys_sets_negkeyset)
{
    RemoteMapState ms;
    ms.initNegKeys(42);

    EXPECT_EQ(42, ms.getNegKeys());
    EXPECT_EQ(true, ms.getNegKeysUsed());
}

TEST(RemoteMapState, initLMouseButton_sets_lMouseButton)
{
    RemoteMapState ms;
    ms.initLMouseButton();

    EXPECT_EQ(true, ms.getLMouseButton());
}

TEST(RemoteMapState, initRMouseButton_sets_RMouseButton)
{
    RemoteMapState ms;
    ms.initRMouseButton();

    EXPECT_EQ(true, ms.getRMouseButton());
}

TEST(RemoteMapState, operator_equals_default_constructed_equal)
{
    RemoteMapState ms1;
    RemoteMapState ms2;

    EXPECT_EQ(ms1, ms2);
    EXPECT_FALSE(ms1 != ms2);
}

TEST(RemoteMapState, operator_equals_all_states_initialized_same_equal)
{
    RemoteMapState ms1;
    ms1.initKeys(1);
    ms1.initNegKeys(2);
    ms1.initLMouseButton();
    ms1.initRMouseButton();
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms1.initRSwitch(Remote::SwitchState::MID);
    RemoteMapState ms2;
    ms2.initKeys(1);
    ms2.initNegKeys(2);
    ms2.initLMouseButton();
    ms2.initRMouseButton();
    ms2.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initRSwitch(Remote::SwitchState::MID);

    EXPECT_EQ(ms1, ms2);
    EXPECT_FALSE(ms1 != ms2);
}

TEST(RemoteMapState, operator_equals_all_states_initialized_not_same_not_equal)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(1);
    ms1.initNegKeys(2);
    ms1.initLMouseButton();
    ms1.initRMouseButton();
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms1.initRSwitch(Remote::SwitchState::MID);
    ms2.initKeys(~1);
    ms2.initNegKeys(~2);
    ms2.initRMouseButton();
    ms2.initLSwitch(Remote::SwitchState::MID);
    ms2.initRSwitch(Remote::SwitchState::UP);

    EXPECT_NE(ms1, ms2);
    EXPECT_TRUE(ms1 != ms2);
}

TEST(RemoteMapState, stateSubsetOf_true_if_default_constructed)
{
    RemoteMapState ms1, ms2;

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

// stateSubsetOf checking key combos

TEST(RemoteMapState, stateSubsetOf_true_keys_equals)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(42);
    ms2.initKeys(42);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_false_keys_not_subset)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(0b1);
    ms2.initKeys(0b10);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_one_way_keys_subset)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(0xffff);
    ms1.initKeys(0x1234);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_one_way_one_keyset_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(42);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_ignores_neg_keys)
{
    RemoteMapState ms1, ms2;
    ms1.initKeys(0b1);
    ms2.initKeys(0b11);
    ms1.initNegKeys(0b100);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

// stateSubsetOf checking mouse buttons

TEST(RemoteMapState, stateSubsetOf_true_one_way_one_left_mouse_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initLMouseButton();

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_left_mice_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initLMouseButton();
    ms2.initLMouseButton();

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_one_way_one_right_mouse_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initRMouseButton();

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_right_mice_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initRMouseButton();
    ms2.initRMouseButton();

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

// stateSubsetOf checking switch states

TEST(RemoteMapState, stateSubsetOf_true_one_way_left_switch_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_left_switches_same)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initLSwitch(Remote::SwitchState::DOWN);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_false_left_switches_different)
{
    RemoteMapState ms1, ms2;
    ms1.initLSwitch(Remote::SwitchState::DOWN);
    ms2.initLSwitch(Remote::SwitchState::MID);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_one_way_right_switch_initialized)
{
    RemoteMapState ms1, ms2;
    ms1.initRSwitch(Remote::SwitchState::DOWN);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_true_right_switches_same)
{
    RemoteMapState ms1, ms2;
    ms1.initRSwitch(Remote::SwitchState::DOWN);
    ms2.initRSwitch(Remote::SwitchState::DOWN);

    EXPECT_TRUE(ms1.stateSubsetOf(ms2));
    EXPECT_TRUE(ms2.stateSubsetOf(ms1));
}

TEST(RemoteMapState, stateSubsetOf_false_right_switches_different)
{
    RemoteMapState ms1, ms2;
    ms1.initRSwitch(Remote::SwitchState::DOWN);
    ms2.initRSwitch(Remote::SwitchState::MID);

    EXPECT_FALSE(ms1.stateSubsetOf(ms2));
    EXPECT_FALSE(ms2.stateSubsetOf(ms1));
}
