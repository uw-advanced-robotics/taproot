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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "aruwlib/Drivers.hpp"

#include "aruwsrc/control/launcher/friction_wheel_rotate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/mock/FrictionWheelSubsystemMock.hpp"

using namespace aruwsrc::launcher;
using aruwlib::Drivers;
using aruwsrc::mock::FrictionWheelSubsystemMock;
using namespace testing;

TEST(FrictionWheelRotateCommand, execute_zero_desired_rpm_always_zero)
{
    Drivers d;
    FrictionWheelSubsystemMock fs(&d);
    FrictionWheelRotateCommand fc(&fs, 0);
    EXPECT_CALL(fs, setDesiredRpm(0));

    fc.execute();
}

TEST(FrictionWheelRotateCommand, execute_positive_rpm_always_positive)
{
    Drivers d;
    FrictionWheelSubsystemMock fs(&d);
    FrictionWheelRotateCommand fc(&fs, 10000);
    EXPECT_CALL(fs, setDesiredRpm(10000));

    fc.execute();
}

TEST(FrictionWheelRotateCommand, execute_negative_rpm_always_negative)
{
    Drivers d;
    FrictionWheelSubsystemMock fs(&d);
    FrictionWheelRotateCommand fc(&fs, -10000);
    EXPECT_CALL(fs, setDesiredRpm(-10000));

    fc.execute();
}

TEST(FrictionWheelRotateCommand, end_resets_desired_rpm_to_zero)
{
    Drivers d;
    FrictionWheelSubsystemMock fs(&d);
    FrictionWheelRotateCommand fc(&fs, 10000);
    InSequence s;
    EXPECT_CALL(fs, setDesiredRpm(10000));
    EXPECT_CALL(fs, setDesiredRpm(0));
    EXPECT_CALL(fs, setDesiredRpm(10000));
    EXPECT_CALL(fs, setDesiredRpm(0));

    fc.execute();
    fc.end(false);
    fc.execute();
    fc.end(true);
}

TEST(FrictionWheelRotateCommand, isFinished_always_false)
{
    Drivers d;
    FrictionWheelSubsystemMock fs(&d);
    FrictionWheelRotateCommand fc(&fs, 10000);
    const int EXECUTE_TIMES = 100;
    EXPECT_CALL(fs, setDesiredRpm(10000)).Times(EXECUTE_TIMES);

    EXPECT_FALSE(fc.isFinished());
    for (int i = 0; i < EXECUTE_TIMES; i++)
    {
        fc.execute();
    }
    EXPECT_FALSE(fc.isFinished());
}
