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

#include "limit_switch_agitator_subsystem.hpp"

#include "aruwlib/drivers.hpp"

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace agitator
{
LimitSwitchAgitatorSubsystem::LimitSwitchAgitatorSubsystem(
    aruwlib::Drivers* drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float distanceTolerance,
    uint32_t temporalTolerance,
    aruwlib::gpio::Digital::InputPin limitSwitchPin)
    : Subsystem(drivers),
      AgitatorSubsystem(
          drivers,
          kp,
          ki,
          kd,
          maxIAccum,
          maxOutput,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          true,
          distanceTolerance,
          temporalTolerance),
      limitSwitchPin(limitSwitchPin),
      digital(&drivers->digital),
      ballsInTube(0)
{
}

void LimitSwitchAgitatorSubsystem::refresh()
{
    if (agitatorIsCalibrated)
    {
        agitatorRunPositionPid();
    }
    else
    {
        calibrateHere();
    }

    const bool newLimitSwitchPressed = !digital->read(limitSwitchPin);
    const float heat42 = drivers->refSerial.getRobotData().turret.heat42;

    // Ball has been fired
    if (heat42 - prevHeat42 > FIRING_HEAT_INCREASE_42 - 10)
    {
        ballsInTube = std::max(0, ballsInTube - 1);
    }

    // Limit switch rising edge
    if (newLimitSwitchPressed && !limitSwitchPressed)
    {
        ballsInTube++;
    }

    prevHeat42 = heat42;
    limitSwitchPressed = newLimitSwitchPressed;
}

}  // namespace agitator

}  // namespace aruwsrc
