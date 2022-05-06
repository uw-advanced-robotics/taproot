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

#ifndef TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_MOCK_HPP_
#define TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/control/velocity/interfaces/velocity_setpoint_subsystem.hpp"

namespace tap::mock
{
/**
 * A class for mocking a setpoint subsystem. Will by default act as if it
 * were unjammed and online (specified in mock_constructors.cpp)
 */
class VelocitySetpointSubsystemMock : public control::velocity::VelocitySetpointSubsystem
{
public:
    VelocitySetpointSubsystemMock(tap::Drivers* drivers);
    virtual ~VelocitySetpointSubsystemMock();

    MOCK_METHOD(float, getVelocitySetpoint, (), (const override));
    MOCK_METHOD(void, setVelocitySetpoint, (float), (override));
    MOCK_METHOD(float, getVelocity, (), (const override));
    MOCK_METHOD(float, getPosition, (), (const override));
    MOCK_METHOD(bool, calibrateHere, (), (override));
    MOCK_METHOD(bool, isJammed, (), (override));
    MOCK_METHOD(void, clearJam, (), (override));
    MOCK_METHOD(bool, isCalibrated, (), (override));
    MOCK_METHOD(bool, isOnline, (), (override));
};

}  // namespace mock

#endif  // TAPROOT_VELOCITY_SETPOINT_SUBSYSTEM_MOCK_HPP_
