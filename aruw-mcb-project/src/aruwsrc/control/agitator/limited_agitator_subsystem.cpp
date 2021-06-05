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

#include "aruwsrc/control/agitator/limited_agitator_subsystem.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwsrc
{
namespace agitator
{
LimitedAgitatorSubsystem::LimitedAgitatorSubsystem(
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
    aruwlib::gpio::Digital::InputPin limitSwitchPin,
    uint8_t debounceMaxSum,
    uint8_t debounceLowerBound,
    uint8_t debounceUpperBound)
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
          isAgitatorInverted),
      limitSwitchPin(limitSwitchPin),
      digital(&drivers->digital),
      debounceFilter(debounceMaxSum, debounceLowerBound, debounceUpperBound)
{
}

void LimitedAgitatorSubsystem::refresh()
{
    if (agitatorIsCalibrated)
    {
        agitatorRunPositionPid();
    }
    else
    {
        agitatorCalibrateHere();
    }

    bool limitSwitchPressed = digital->read(limitSwitchPin);
    debounceFilter.update(limitSwitchPressed);
}

bool LimitedAgitatorSubsystem::isLimitSwitchPressed() const { return debounceFilter.getValue(); }

}  // namespace agitator

}  // namespace aruwsrc
