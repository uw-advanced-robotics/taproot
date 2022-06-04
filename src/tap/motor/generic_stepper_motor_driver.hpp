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
#ifndef TAPROOT_GENERIC_STEPPER_MOTOR_DRIVER_HPP_
#define TAPROOT_GENERIC_STEPPER_MOTOR_DRIVER_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "stepper_motor_interface.hpp"

namespace tap
{
class Drivers;
}
namespace tap::motor
{
/**
 * A class designed to interface with generic stepper motor controllers
 * that have a direction and pulse pin. Takes a step every time refresh() is called if
 * current position does not match ideal position.
 */
class GenericStepperMotorDriver : public StepperMotorInterface
{
public:
    /**
     * Construct a new Generic Stepper Motor Driver object.
     */
    GenericStepperMotorDriver(
        Drivers* drivers,
        tap::gpio::Digital::OutputPin direction,
        tap::gpio::Digital::OutputPin pulse);

    /**
     * Run driver. Takes a step towards desired position if not there yet when called.
     */
    void refresh() override;

private:
    Drivers* drivers;

    tap::gpio::Digital::OutputPin direction;
    tap::gpio::Digital::OutputPin pulse;

    /**
     * Last set state of the pin. Step pin should be set to opposite of this on next step to
     * take step
     */
    bool pinState;
};

}  // namespace tap::motor
#endif  // TAPROOT_GENERIC_STEPPER_MOTOR_DRIVER_HPP_
