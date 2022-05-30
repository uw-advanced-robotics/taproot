/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "generic_stepper_motor_driver.hpp"

#include "aruwsrc/drivers_singleton.hpp"

namespace tap::motor
{
GenericStepperMotorDriver::GenericStepperMotorDriver(
    Drivers* drivers,
    tap::gpio::Digital::OutputPin direction,
    tap::gpio::Digital::OutputPin pulse)
    : drivers(drivers),
      direction(direction),
      pulse(pulse)
{
}

void GenericStepperMotorDriver::refresh()
{
    if (desiredPosition > position)
    {
        drivers->digital.set(direction, true);
        drivers->digital.set(pulse, !pinState);
        pinState = !pinState;
        position += 1;
    }
    else if (desiredPosition < position)
    {
        drivers->digital.set(direction, false);
        drivers->digital.set(pulse, !pinState);
        pinState = !pinState;
        position -= 1;
    } 
}

}  // namespace tap::motor
