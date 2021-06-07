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

#ifndef CHASSIS_SUBSYSTEM_MOCK_HPP_
#define CHASSIS_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class ChassisSubsystemMock : public aruwsrc::chassis::ChassisSubsystem
{
public:
    ChassisSubsystemMock(aruwlib::Drivers *drivers);
    virtual ~ChassisSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredOutput, (float x, float y, float z), (override));
    MOCK_METHOD(float, chassisSpeedRotationPID, (float currentAngleError, float kp), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(
        float,
        calculateRotationTranslationalGain,
        (float chassisRotationDesiredWheelspeed),
        ());
    MOCK_METHOD(float, getChassisDesiredRotation, (), (const override));
    MOCK_METHOD(int16_t, getLeftFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getLeftBackRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightBackRpmActual, (), (const override));
};  // class ChassisSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // CHASSIS_SUBSYSTEM_MOCK_HPP_
