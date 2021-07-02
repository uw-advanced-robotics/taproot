/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Note: These tests do not fully test any underlying filter structures
 * For example, there are no long term tests to ensure linear interpolation
 * is done properly for the chassis remote input. Instead, it is expected
 * that additional tests should be done to ensure linear interpolation is
 * done properly.
 */

#include <gtest/gtest.h>

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/control/control_operator_interface.hpp"
#include "aruwlib/drivers.hpp"

using aruwlib::Drivers;
using aruwlib::Remote;
using aruwlib::control::ControlOperatorInterface;
using namespace testing;
using namespace aruwlib::arch::clock;
using namespace aruwlib::algorithms;

static constexpr float MAX_REMOTE = 1.0f;

#define INIT_TEST    \
    Drivers drivers; \
    ControlOperatorInterface operatorInterface(&drivers);

static float runChassisXInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteVal,
    bool wPressed,
    bool sPressed)
{
    EXPECT_CALL(drivers.remote, getUpdateCounter).WillOnce(Return(1));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::W)).WillOnce(Return(wPressed));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::S)).WillOnce(Return(sPressed));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL))
        .WillOnce(Return(remoteVal));
    return operatorInterface.getChassisXInput();
}

static float runChassisYInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteVal,
    bool dPressed,
    bool aPressed)
{
    EXPECT_CALL(drivers.remote, getUpdateCounter).WillOnce(Return(1));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::A)).WillOnce(Return(aPressed));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::D)).WillOnce(Return(dPressed));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_HORIZONTAL))
        .WillOnce(Return(remoteVal));
    return operatorInterface.getChassisYInput();
}

static float runChassisRInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteVal,
    bool qPressed,
    bool ePressed)
{
    EXPECT_CALL(drivers.remote, getUpdateCounter).WillOnce(Return(1));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::Q)).WillOnce(Return(qPressed));
    EXPECT_CALL(drivers.remote, keyPressed(Remote::Key::E)).WillOnce(Return(ePressed));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillOnce(Return(remoteVal));
    return operatorInterface.getChassisRInput();
}

static float runTurretYawInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteInput,
    int16_t mouseInput)
{
    EXPECT_CALL(drivers.remote, getMouseX).WillOnce(Return(mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillOnce(Return(remoteInput));
    return operatorInterface.getTurretYawInput();
}

static float runTurretPitchInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteInput,
    int16_t mouseInput)
{
    EXPECT_CALL(drivers.remote, getMouseY).WillOnce(Return(mouseInput));
    EXPECT_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_VERTICAL))
        .WillOnce(Return(remoteInput));
    return operatorInterface.getTurretPitchInput();
}

static float runSentinelChassisInputTest(
    Drivers &drivers,
    ControlOperatorInterface &operatorInterface,
    float remoteInput)
{
    EXPECT_CALL(drivers.remote, getChannel).WillOnce(Return(remoteInput));
    return operatorInterface.getSentinelSpeedInput();
}

// Tests run x, y and r input at the same time, testing similar input parameters.

TEST(ControlOperatorInterface, getChassisInput_zeros)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(0, runChassisXInputTest(drivers, operatorInterface, 0, false, false));
    EXPECT_FLOAT_EQ(0, runChassisYInputTest(drivers, operatorInterface, 0, false, false));
    EXPECT_FLOAT_EQ(0, runChassisRInputTest(drivers, operatorInterface, 0, false, false));
}

TEST(ControlOperatorInterface, getChassisInput_min_key_user_input_limited)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_X_KEY_INPUT_FILTER_ALPHA,
        runChassisXInputTest(drivers, operatorInterface, 0, false, true));
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_Y_KEY_INPUT_FILTER_ALPHA,
        runChassisYInputTest(drivers, operatorInterface, 0, false, true));
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_R_KEY_INPUT_FILTER_ALPHA,
        runChassisRInputTest(drivers, operatorInterface, 0, false, true));
}

TEST(ControlOperatorInterface, getChassisInput_max_key_user_input_limited)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        ControlOperatorInterface::CHASSIS_X_KEY_INPUT_FILTER_ALPHA,
        runChassisXInputTest(drivers, operatorInterface, 0, true, false));
    EXPECT_FLOAT_EQ(
        ControlOperatorInterface::CHASSIS_Y_KEY_INPUT_FILTER_ALPHA,
        runChassisYInputTest(drivers, operatorInterface, 0, true, false));
    EXPECT_FLOAT_EQ(
        ControlOperatorInterface::CHASSIS_R_KEY_INPUT_FILTER_ALPHA,
        runChassisRInputTest(drivers, operatorInterface, 0, true, false));
}

TEST(ControlOperatorInterface, getChassisInput_min_and_max_keys_cancel)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(0, runChassisXInputTest(drivers, operatorInterface, 0, true, true));
    EXPECT_FLOAT_EQ(0, runChassisYInputTest(drivers, operatorInterface, 0, true, true));
    EXPECT_FLOAT_EQ(0, runChassisRInputTest(drivers, operatorInterface, 0, true, true));
}

TEST(ControlOperatorInterface, getChassisInput_min_remote_limited)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        -1,
        runChassisXInputTest(drivers, operatorInterface, -MAX_REMOTE - 10, false, false));
    EXPECT_FLOAT_EQ(
        -1,
        runChassisYInputTest(drivers, operatorInterface, -MAX_REMOTE - 10, false, false));
    EXPECT_FLOAT_EQ(
        -1,
        runChassisRInputTest(drivers, operatorInterface, -MAX_REMOTE - 10, false, false));
}

TEST(ControlOperatorInterface, getChassisInput_max_remote_limited)
{
    INIT_TEST
    setTime(1);

    EXPECT_FLOAT_EQ(
        1,
        runChassisXInputTest(drivers, operatorInterface, MAX_REMOTE + 10, false, false));
    EXPECT_FLOAT_EQ(
        1,
        runChassisYInputTest(drivers, operatorInterface, MAX_REMOTE + 10, false, false));
    EXPECT_FLOAT_EQ(
        1,
        runChassisRInputTest(drivers, operatorInterface, MAX_REMOTE + 10, false, false));
}

TEST(ControlOperatorInterface, getChassisInput_half_min_remote_half_min_output)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        -0.5f,
        runChassisXInputTest(drivers, operatorInterface, -MAX_REMOTE / 2.0f, false, false));
    EXPECT_FLOAT_EQ(
        -0.5f,
        runChassisYInputTest(drivers, operatorInterface, -MAX_REMOTE / 2.0f, false, false));
    EXPECT_FLOAT_EQ(
        -0.5f,
        runChassisRInputTest(drivers, operatorInterface, -MAX_REMOTE / 2.0f, false, false));
}

TEST(ControlOperatorInterface, getChassisInput_half_max_remote_half_max_output)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisXInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, false, false));
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisYInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, false, false));
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisRInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, false, false));
}

TEST(ControlOperatorInterface, getChassisInput_max_remote_and_min_key_pressed_cancels)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_X_KEY_INPUT_FILTER_ALPHA + MAX_REMOTE,
        runChassisXInputTest(drivers, operatorInterface, MAX_REMOTE, false, true));
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_Y_KEY_INPUT_FILTER_ALPHA + MAX_REMOTE,
        runChassisYInputTest(drivers, operatorInterface, MAX_REMOTE, false, true));
    EXPECT_FLOAT_EQ(
        -ControlOperatorInterface::CHASSIS_R_KEY_INPUT_FILTER_ALPHA + MAX_REMOTE,
        runChassisRInputTest(drivers, operatorInterface, MAX_REMOTE, false, true));
}

TEST(ControlOperatorInterface, getChassisInput_half_max_remote_and_max_and_min_key_pressed_cancels)
{
    INIT_TEST
    setTime(1);
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisXInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, true, true));
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisYInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, true, true));
    EXPECT_FLOAT_EQ(
        0.5f,
        runChassisRInputTest(drivers, operatorInterface, MAX_REMOTE / 2.0f, true, true));
}

// Note: Remote input inverted for yaw control.

TEST(ControlOperatorInterface, getTurretInput_zeros)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(0, runTurretYawInputTest(drivers, operatorInterface, 0, 0));
    EXPECT_FLOAT_EQ(0, runTurretPitchInputTest(drivers, operatorInterface, 0, 0));
}

TEST(ControlOperatorInterface, getTurretInput_min_remote_input_limited)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(1, runTurretYawInputTest(drivers, operatorInterface, -1, 0));
    EXPECT_FLOAT_EQ(-1, runTurretPitchInputTest(drivers, operatorInterface, -1, 0));
}

TEST(ControlOperatorInterface, getTurretInput_max_remote_input_limited)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(-1, runTurretYawInputTest(drivers, operatorInterface, 1, 0));
    EXPECT_FLOAT_EQ(1, runTurretPitchInputTest(drivers, operatorInterface, 1, 0));
}

TEST(ControlOperatorInterface, getTurretInput_range_of_remote_input_directly_maps_to_output)
{
    INIT_TEST
    for (float i = -1; i < 1; i += 0.1f)
    {
        EXPECT_FLOAT_EQ(-i, runTurretYawInputTest(drivers, operatorInterface, i, 0));
        EXPECT_FLOAT_EQ(i, runTurretPitchInputTest(drivers, operatorInterface, i, 0));
    }
}

TEST(ControlOperatorInterface, getTurretInput_min_mouse_limited)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(1, runTurretYawInputTest(drivers, operatorInterface, 0, INT16_MIN + 1));
    EXPECT_FLOAT_EQ(1, runTurretPitchInputTest(drivers, operatorInterface, 0, INT16_MIN + 1));

    EXPECT_FLOAT_EQ(
        1,
        runTurretYawInputTest(
            drivers,
            operatorInterface,
            0,
            -ControlOperatorInterface::USER_MOUSE_YAW_MAX - 100));
    EXPECT_FLOAT_EQ(
        1,
        runTurretPitchInputTest(
            drivers,
            operatorInterface,
            0,
            -ControlOperatorInterface::USER_MOUSE_PITCH_MAX - 100));

    EXPECT_FLOAT_EQ(
        1,
        runTurretYawInputTest(
            drivers,
            operatorInterface,
            0,
            -ControlOperatorInterface::USER_MOUSE_YAW_MAX));
    EXPECT_FLOAT_EQ(
        1,
        runTurretPitchInputTest(
            drivers,
            operatorInterface,
            0,
            -ControlOperatorInterface::USER_MOUSE_PITCH_MAX));
}

TEST(ControlOperatorInterface, getTurretInput_max_mouse_limited)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(-1, runTurretYawInputTest(drivers, operatorInterface, 0, INT16_MAX));
    EXPECT_FLOAT_EQ(-1, runTurretPitchInputTest(drivers, operatorInterface, 0, INT16_MAX));

    EXPECT_FLOAT_EQ(
        -1,
        runTurretYawInputTest(
            drivers,
            operatorInterface,
            0,
            ControlOperatorInterface::USER_MOUSE_YAW_MAX + 100));
    EXPECT_FLOAT_EQ(
        -1,
        runTurretPitchInputTest(
            drivers,
            operatorInterface,
            0,
            ControlOperatorInterface::USER_MOUSE_PITCH_MAX + 100));

    EXPECT_FLOAT_EQ(
        -1,
        runTurretYawInputTest(
            drivers,
            operatorInterface,
            0,
            ControlOperatorInterface::USER_MOUSE_YAW_MAX));
    EXPECT_FLOAT_EQ(
        -1,
        runTurretPitchInputTest(
            drivers,
            operatorInterface,
            0,
            ControlOperatorInterface::USER_MOUSE_PITCH_MAX));
}

TEST(
    ControlOperatorInterface,
    getTurretInput_range_of_mouse_mappings_directly_maps_to_output_within_min_max_boundary)
{
    INIT_TEST
    for (int16_t i = -ControlOperatorInterface::USER_MOUSE_YAW_MAX;
         i < ControlOperatorInterface::USER_MOUSE_YAW_MAX;
         i += 10)
    {
        EXPECT_FLOAT_EQ(
            static_cast<float>(i) / ControlOperatorInterface::USER_MOUSE_YAW_MAX,
            runTurretYawInputTest(drivers, operatorInterface, 0, -i));
    }
    for (int16_t i = -ControlOperatorInterface::USER_MOUSE_PITCH_MAX;
         i < ControlOperatorInterface::USER_MOUSE_PITCH_MAX;
         i += 10)
    {
        EXPECT_FLOAT_EQ(
            -static_cast<float>(i) / ControlOperatorInterface::USER_MOUSE_PITCH_MAX,
            runTurretPitchInputTest(drivers, operatorInterface, 0, i));
    }
}

TEST(ControlOperatorInterface, getTurretInput_mouse_and_remote_mappings_additive)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(
        -1,
        runTurretYawInputTest(
            drivers,
            operatorInterface,
            MAX_REMOTE / 2.0f,
            ControlOperatorInterface::USER_MOUSE_YAW_MAX / 2.0f));
    EXPECT_FLOAT_EQ(
        1,
        runTurretPitchInputTest(
            drivers,
            operatorInterface,
            MAX_REMOTE / 2.0f,
            -ControlOperatorInterface::USER_MOUSE_PITCH_MAX / 2.0f));
}

TEST(ControlOperatorInterface, getSentinelSpeedInput_zeros)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(0, runSentinelChassisInputTest(drivers, operatorInterface, 0));
}

TEST(ControlOperatorInterface, getSentinelSpeedInput_min_remote)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(
        -MAX_REMOTE * ControlOperatorInterface::USER_STICK_SENTINEL_DRIVE_SCALAR,
        runSentinelChassisInputTest(drivers, operatorInterface, -MAX_REMOTE));
}

TEST(ControlOperatorInterface, getSentinelSpeedInput_max_remote)
{
    INIT_TEST
    EXPECT_FLOAT_EQ(
        MAX_REMOTE * ControlOperatorInterface::USER_STICK_SENTINEL_DRIVE_SCALAR,
        runSentinelChassisInputTest(drivers, operatorInterface, MAX_REMOTE));
}

TEST(ControlOperatorInterface, getSentinelSpeedInput_complete_remote_range_valid)
{
    INIT_TEST
    for (float i = -MAX_REMOTE; i < MAX_REMOTE; i += 0.1f)
    {
        EXPECT_FLOAT_EQ(
            i * ControlOperatorInterface::USER_STICK_SENTINEL_DRIVE_SCALAR,
            runSentinelChassisInputTest(drivers, operatorInterface, i));
    }
}
