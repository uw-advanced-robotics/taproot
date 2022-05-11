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

#ifndef STEPPER_MOTOR_INTERFACE_HPP_
#define STEPPER_MOTOR_INTERFACE_HPP_

#include <cstdint>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "dji_motor.hpp"
#include "motor_interface.hpp"

namespace tap::motor
{
class StepperMotor : public MotorInterface
{
public:
    StepperMotor(Drivers* drivers, bool motorInverted, const char* name, tap::gpio::Digital::OutputPin direction, tap::gpio::Pwm::Pin pulse);

    void initialize() override;

    int64_t getEncoderUnwrapped() const override;
    uint16_t getEncoderWrapped() const override;
    // controls the stepper in some way (take a step)
    void setDesiredOutput(int32_t desiredOutput) override;
    bool isMotorOnline() const override;
    // return -1 if going backwards, 0 if not moving 1 if forwards
    int16_t getOutputDesired() const override;
    int8_t getTemperature() const override;
    int16_t getTorque() const override;
    int16_t getShaftRPM() const override;

private:
    Drivers* drivers;

    int16_t desiredOutput;

    const char* motorName;

    /**
     * If `false` the positive rotation direction of the shaft is counter-clockwise when
     * looking at the shaft from the side opposite the motor. If `true` then the positive
     * rotation direction will be clockwise.
     */
    bool motorInverted;

    tap::gpio::Digital::OutputPin direction;
    tap::gpio::Pwm::Pin pulse;
};
}  // namespace tap::motor

#endif  // STEPPER_MOTOR_HPP_
