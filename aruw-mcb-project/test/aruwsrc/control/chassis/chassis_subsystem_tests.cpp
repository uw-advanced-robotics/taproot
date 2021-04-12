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

#if defined(TARGET_SOLDIER)

#include <aruwlib/Drivers.hpp>
#include <gtest/gtest.h>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

using aruwlib::Drivers;
using aruwlib::algorithms::PI;
using modm::Matrix;
using namespace aruwsrc::chassis;

static constexpr float WHEEL_VEL = 1000;
static constexpr float CHASSIS_VEL = 0.41887906;
static constexpr float CHASSIS_VEL_R = 0.15330973;

static void setDefaultExpectations(Drivers *d)
{
    EXPECT_CALL(d->djiMotorTxHandler, removeFromMotorManager).Times(4);
    EXPECT_CALL(d->canRxHandler, removeReceiveHandler).Times(4);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_zero_desired_output)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, 0, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_x_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(WHEEL_VEL, 0, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_r_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, 0, WHEEL_VEL);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_x_and_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(WHEEL_VEL, WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_x_y_and_r_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(WHEEL_VEL, WHEEL_VEL, WHEEL_VEL);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_negative_x_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(-WHEEL_VEL, 0, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_negative_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, -WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_negative_r_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, 0, -WHEEL_VEL);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL_R, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_negative_x_and_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(-WHEEL_VEL, -WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_negative_x_and_positive_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(-WHEEL_VEL, WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getDesiredVelocityChassisRelative_positive_x_and_negative_y_output_desired)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(WHEEL_VEL, -WHEEL_VEL, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

// Need motor mock + need to decide how to mock in-place motors before tests can be written
TEST(ChassisSubsystem, getActualVelocityChassisRelative) {}

TEST(ChassisSubsystem, getVelocityWorldRelative_zero_desired_output_with_any_heading)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, 0, 0);
    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 0);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
    cs.getVelocityWorldRelative(chassisVelocity, PI / 4.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
    cs.getVelocityWorldRelative(chassisVelocity, PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
    cs.getVelocityWorldRelative(chassisVelocity, 3.0f * PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getVelocityWorldRelative_x_different_headings)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(WHEEL_VEL, 0, 0);

    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 0);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 4.0f);
    EXPECT_FLOAT_EQ(CHASSIS_VEL * cosf(PI / 4.0f), chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL * sinf(PI / 4.0f), chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 3.0f * PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getVelocityWorldRelative_y_different_headings)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, WHEEL_VEL, 0);

    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 0);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 4.0f);
    EXPECT_FLOAT_EQ(CHASSIS_VEL * cosf(3.0f * PI / 4.0f), chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL * sinf(3.0f * PI / 4.0f), chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 2.0f);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(-CHASSIS_VEL, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 3.0f * PI / 2.0f);
    EXPECT_FLOAT_EQ(CHASSIS_VEL, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[2][0]);
}

TEST(ChassisSubsystem, getVelocityWorldRelative_r_different_headings)
{
    Drivers d;
    setDefaultExpectations(&d);
    ChassisSubsystem cs(&d);

    cs.setDesiredOutput(0, 0, WHEEL_VEL);

    Matrix<float, 3, 1> chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 0);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 4.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, PI);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);

    chassisVelocity = cs.getDesiredVelocityChassisRelative();
    cs.getVelocityWorldRelative(chassisVelocity, 3.0f * PI / 2.0f);
    EXPECT_FLOAT_EQ(0, chassisVelocity[0][0]);
    EXPECT_FLOAT_EQ(0, chassisVelocity[1][0]);
    EXPECT_FLOAT_EQ(CHASSIS_VEL_R, chassisVelocity[2][0]);
}

#endif
