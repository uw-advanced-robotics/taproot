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

#ifndef AGITATOR_SUBSYSTEM_MOCK_HPP_
#define AGITATOR_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class AgitatorSubsystemMock : public agitator::AgitatorSubsystem
{
public:
    AgitatorSubsystemMock(
        aruwlib::Drivers* drivers,
        float kp = 0,
        float ki = 0,
        float kd = 0,
        float maxIAccum = 0,
        float maxOutput = 0,
        float agitatorGearRatio = 0,
        aruwlib::motor::MotorId agitatorMotorId = aruwlib::motor::MOTOR7,
        aruwlib::can::CanBus agitatorCanBusId = aruwlib::can::CanBus::CAN_BUS1,
        bool isAgitatorInverted = false);
    virtual ~AgitatorSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));

    MOCK_METHOD(void, refresh, (), (override));

    MOCK_METHOD(void, setAgitatorDesiredAngle, (float newAngle), (override));

    MOCK_METHOD(float, getAgitatorAngle, (), (const, override));

    MOCK_METHOD(float, getAgitatorDesiredAngle, (), (const, override));

    MOCK_METHOD(bool, agitatorCalibrateHere, (), (override));

    MOCK_METHOD(void, armAgitatorUnjamTimer, (uint32_t predictedRotateTime), (override));

    MOCK_METHOD(void, disarmAgitatorUnjamTimer, (), (override));

    MOCK_METHOD(bool, isAgitatorJammed, (), (const, override));

    MOCK_METHOD(bool, isAgitatorCalibrated, (), (const, override));

    MOCK_METHOD(bool, isAgitatorOnline, (), (const, override));

    MOCK_METHOD(float, getAgitatorVelocity, (), (const, override));

    MOCK_METHOD(const char*, getName, (), (override));
};  // class AgitatorSubsystemMock

}  // namespace mock

}  // namespace aruwsrc

#endif  // AGITATOR_SUBSYSTEM_MOCK_HPP_
