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

#include "AgitatorSubsystemMock.hpp"
#include "BeybladeCommandMock.hpp"
#include "ChassisDriveCommandMock.hpp"
#include "ChassisSubsystemMock.hpp"
#include "FrictionWheelSubsystemMock.hpp"
#include "GrabberSubsystemMock.hpp"
#include "HopperSubsystemMock.hpp"
#include "SentinelDriveSubsystemMock.hpp"
#include "TowSubsystemMock.hpp"
#include "TurretSubsystemMock.hpp"
#include "XAxisSubsystemMock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace aruwsrc::mock
{
AgitatorSubsystemMock::AgitatorSubsystemMock(
    aruwlib::Drivers *drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted)
    : AgitatorSubsystem(
          drivers,
          kp,
          ki,
          kd,
          maxIAccum,
          maxOutput,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted)
{
}
AgitatorSubsystemMock::~AgitatorSubsystemMock() {}

BeybladeCommandMock::BeybladeCommandMock(
    aruwlib::Drivers *drivers,
    chassis::ChassisSubsystem *chassis,
    turret::TurretSubsystem *turret)
    : BeybladeCommand(drivers, chassis, turret)
{
}
BeybladeCommandMock::~BeybladeCommandMock() {}

ChassisDriveCommandMock::ChassisDriveCommandMock(aruwlib::Drivers *d, chassis::ChassisSubsystem *cs)
    : chassis::ChassisDriveCommand(d, cs)
{
}
ChassisDriveCommandMock::~ChassisDriveCommandMock() {}

ChassisSubsystemMock::ChassisSubsystemMock(aruwlib::Drivers *drivers) : ChassisSubsystem(drivers) {}
ChassisSubsystemMock::~ChassisSubsystemMock() {}

FrictionWheelSubsystemMock::FrictionWheelSubsystemMock(aruwlib::Drivers *drivers)
    : FrictionWheelSubsystem(drivers)
{
}
FrictionWheelSubsystemMock::~FrictionWheelSubsystemMock() {}

GrabberSubsystemMock::GrabberSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin pin)
    : engineer::GrabberSubsystem(drivers, pin)
{
}
GrabberSubsystemMock::~GrabberSubsystemMock() {}

HopperSubsystemMock::HopperSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Pwm::Pin pwmPin,
    float open,
    float close,
    float pwmRampSpeed)
    : control::HopperSubsystem(drivers, pwmPin, open, close, pwmRampSpeed)
{
}
HopperSubsystemMock::~HopperSubsystemMock() {}

SentinelDriveSubsystemMock::SentinelDriveSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::InputPin leftLimitSwitch,
    aruwlib::gpio::Digital::InputPin rightLimitSwitch)
    : control::SentinelDriveSubsystem(drivers, leftLimitSwitch, rightLimitSwitch)
{
}
SentinelDriveSubsystemMock::~SentinelDriveSubsystemMock() {}

TowSubsystemMock::TowSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin leftTowPin,
    aruwlib::gpio::Digital::OutputPin rightTowPin,
    aruwlib::gpio::Digital::InputPin leftTowLimitSwitchPin,
    aruwlib::gpio::Digital::InputPin rightTowLimitSwitchPin)
    : aruwsrc::engineer::TowSubsystem(
          drivers,
          leftTowPin,
          rightTowPin,
          leftTowLimitSwitchPin,
          rightTowLimitSwitchPin)
{
}
TowSubsystemMock::~TowSubsystemMock() {}

TurretSubsystemMock::TurretSubsystemMock(aruwlib::Drivers *drivers) : TurretSubsystem(drivers) {}
TurretSubsystemMock::~TurretSubsystemMock() {}

XAxisSubsystemMock::XAxisSubsystemMock(
    aruwlib::Drivers *drivers,
    aruwlib::gpio::Digital::OutputPin pin)
    : engineer::XAxisSubsystem(drivers, pin)
{
}
XAxisSubsystemMock::~XAxisSubsystemMock() {}
}  // namespace aruwsrc::mock
