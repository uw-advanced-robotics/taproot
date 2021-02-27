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

#ifndef __FRICTION_WHEEL_SUBSYSTEM_HPP__
#define __FRICTION_WHEEL_SUBSYSTEM_HPP__

#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include <aruwlib/mock/DJIMotorMock.hpp>
#else
#include <aruwlib/motor/dji_motor.hpp>
#endif

#include <modm/math/filter/pid.hpp>

#include "util_macros.hpp"

namespace aruwsrc
{
namespace launcher
{
/**
 * A subsystem which regulates the speed of a two wheel shooter system.
 */
class FrictionWheelSubsystem : public aruwlib::control::Subsystem
{
public:
    /**
     * Creates a new friction wheel subsystem with DJI motor1 and motor2
     * unless otherwise specified on CAN bus 1.
     */
    FrictionWheelSubsystem(
        aruwlib::Drivers *drivers,
        aruwlib::motor::MotorId leftMotorId = LEFT_MOTOR_ID,
        aruwlib::motor::MotorId rightMotorId = RIGHT_MOTOR_ID)
        : aruwlib::control::Subsystem(drivers),
          velocityPidLeftWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          velocityPidRightWheel(PID_P, PID_I, PID_D, PID_MAX_ERROR_SUM, PID_MAX_OUTPUT),
          desiredRpmRamp(0),
          leftWheel(drivers, leftMotorId, CAN_BUS_MOTORS, true, "left example motor"),
          rightWheel(drivers, rightMotorId, CAN_BUS_MOTORS, false, "right example motor")
    {
    }

    void initialize() override;

    /**
     * Sets target flywheel RPM.
     */
    mockable void setDesiredRpm(float desRpm);

    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override;

    void runHardwareTests() override;

    const char *getName() override { return "Friction Wheel"; }

private:
    static constexpr aruwlib::motor::MotorId LEFT_MOTOR_ID = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::motor::MotorId RIGHT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;

    // speed of ramp when you set a new desired ramp speed [rpm / ms]
    static constexpr float FRICTION_WHEEL_RAMP_SPEED = 1.0f;

    static constexpr float PID_P = 30.0f;
    static constexpr float PID_I = 0.0f;
    static constexpr float PID_D = 5.0f;
    static constexpr float PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 16000.0f;

    modm::Pid<float> velocityPidLeftWheel;

    modm::Pid<float> velocityPidRightWheel;

    aruwlib::algorithms::Ramp desiredRpmRamp;

    uint32_t prevTime = 0;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock leftWheel;
    aruwlib::mock::DjiMotorMock rightWheel;

private:
#else
    aruwlib::motor::DjiMotor leftWheel;
    aruwlib::motor::DjiMotor rightWheel;
#endif
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
