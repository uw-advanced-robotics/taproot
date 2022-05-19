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

#include "stepper_motor_interface.hpp"
#include "tap/control/subsystem.hpp"

namespace tap {
    class Drivers;
}
namespace tap::motor
{
/**
 * A class designed to interface with generic stepper motor controllers
 * that have a direction and pulse pin. Takes steps at a constant speed.
 */
class GenericStepperMotorDriver : public StepperMotorInterface, public tap::control::Subsystem
{
    public:
    /**
     * Construct a new Generic Stepper Motor Driver object.
     */
    GenericStepperMotorDriver(Drivers* drivers);

    void refresh() override;

        

private:
    Drivers* drivers;
};
}  // namespace tap::motor
#endif // TAPROOT_GENERIC_STEPPER_MOTOR_DRIVER_HPP_
