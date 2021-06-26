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

#include "sentinel_drive_subsystem.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

using namespace aruwlib::gpio;

namespace aruwsrc::control::sentinel::drive
{
SentinelDriveSubsystem::SentinelDriveSubsystem(
    aruwlib::Drivers* drivers,
    aruwlib::gpio::Digital::InputPin leftLimitSwitch,
    aruwlib::gpio::Digital::InputPin rightLimitSwitch,
    aruwlib::motor::MotorId leftMotorId,
    aruwlib::motor::MotorId rightMotorId,
    aruwlib::gpio::Analog::Pin currentSensorPin)
    : aruwlib::control::chassis::IChassisSubsystem(drivers),
      leftLimitSwitch(leftLimitSwitch),
      rightLimitSwitch(rightLimitSwitch),
      velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
      desiredRpm(0),
      leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, false, "left sentinel drive motor"),
      rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right sentinel drive motor"),
      powerLimiter(
          drivers,
          currentSensorPin,
          MAX_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD,
          POWER_CONSUMPTION_THRESHOLD,
          CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING,
          motorConstants)
{
    chassisMotors[0] = &leftWheel;
    chassisMotors[1] = &rightWheel;
}

void SentinelDriveSubsystem::initialize()
{
    drivers->digital.configureInputPullMode(
        leftLimitSwitch,
        aruwlib::gpio::Digital::InputPullMode::PullDown);
    drivers->digital.configureInputPullMode(
        rightLimitSwitch,
        aruwlib::gpio::Digital::InputPullMode::PullDown);
    leftWheel.initialize();
    rightWheel.initialize();
}

void SentinelDriveSubsystem::setDesiredRpm(float desRpm) { desiredRpm = desRpm; }

void SentinelDriveSubsystem::refresh()
{
    velocityPidLeftWheel.update(desiredRpm - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(velocityPidLeftWheel.getValue());
    velocityPidRightWheel.update(desiredRpm - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(velocityPidRightWheel.getValue());
    powerLimiter.performPowerLimiting(chassisMotors, MODM_ARRAY_SIZE(chassisMotors));
    // constantly poll the limit switches, resetting offset if needed
    resetOffsetFromLimitSwitch();
}

float SentinelDriveSubsystem::absolutePosition()
{
    float leftPosition = distanceFromEncoder(&leftWheel) - leftWheelZeroRailOffset;
    float rightPosition = distanceFromEncoder(&rightWheel) - rightWheelZeroRailOffset;
    float average = 0.0f;
    if (leftWheel.isMotorOnline() && rightWheel.isMotorOnline())
    {
        average = (leftPosition + rightPosition) / 2.0f;
    }
    else if (leftWheel.isMotorOnline())
    {
        RAISE_ERROR(
            drivers,
            "right sentinel drive motor offline",
            aruwlib::errors::Location::SUBSYSTEM,
            aruwlib::errors::SubsystemErrorType::MOTOR_OFFLINE);
        average = leftPosition;
    }
    else if (rightWheel.isMotorOnline())
    {
        RAISE_ERROR(
            drivers,
            "left sentinel drive motor offline",
            aruwlib::errors::Location::SUBSYSTEM,
            aruwlib::errors::SubsystemErrorType::MOTOR_OFFLINE);
        average = rightPosition;
    }
    else
    {
        RAISE_ERROR(
            drivers,
            "both sentinel drive motors offline",
            aruwlib::errors::Location::SUBSYSTEM,
            aruwlib::errors::SubsystemErrorType::MOTOR_OFFLINE);
        average = 0.0f;
    }
    return average;
}

// Resets the encoder offset used to determine position of the sentinel on the rail depending on
// which limit switch is hit. If neither limit switch is hit, no-op. Left limit switch indicates
// being at the start of the rail, right limit switch indicates end of rail.
void SentinelDriveSubsystem::resetOffsetFromLimitSwitch()
{
    // DigitalPin where limit switch is placed

    // Note: the left limit switch is active low
    if (!drivers->digital.read(leftLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel);
        rightWheelZeroRailOffset = distanceFromEncoder(&rightWheel);
    }
    else if (drivers->digital.read(rightLimitSwitch))
    {
        leftWheelZeroRailOffset = distanceFromEncoder(&leftWheel) - RAIL_LENGTH + SENTINEL_LENGTH;
        rightWheelZeroRailOffset = distanceFromEncoder(&rightWheel) - RAIL_LENGTH + SENTINEL_LENGTH;
    }
}

// Returns the distance covered by the sentinel wheel on the rail
// with respect to the encoders
// Equation used: Arc Length = Angle * numberOfRotations * radius
// Here we get the radius from the getEncoderUnwrapped function
float SentinelDriveSubsystem::distanceFromEncoder(aruwlib::motor::DjiMotor* motor)
{
    float unwrappedAngle = motor->getEncoderUnwrapped();
    float numberOfRotations = unwrappedAngle / (aruwlib::motor::DjiMotor::ENC_RESOLUTION);
    return numberOfRotations * 2.0f * aruwlib::algorithms::PI * WHEEL_RADIUS / GEAR_RATIO;
}

void SentinelDriveSubsystem::runHardwareTests()
{
    if (abs(rightWheel.getShaftRPM()) > 50.0f) this->setHardwareTestsComplete();
}

void SentinelDriveSubsystem::onHardwareTestStart() { this->setDesiredRpm(100.0f); }

void SentinelDriveSubsystem::onHardwareTestComplete() { this->setDesiredRpm(0.0f); }

}  // namespace aruwsrc::control::sentinel::drive
