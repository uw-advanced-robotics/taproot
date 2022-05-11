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

#ifndef STEPPER_MOTOR_INTERFACE_HPP_
#define STEPPER_MOTOR_INTERFACE_HPP_

#include <cstdint>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#include "dji_motor.hpp"
#include "motor_interface.hpp"

namespace tap::motor
{
/**
 * A class designed to interface with stepper motor controllers.
 */
class StepperMotor : public MotorInterface
{
public:
    /**
     * Construct a new Stepper Motor object
     *
     * @param drivers a pointer to the drivers struct
     * @param motorInverted if 'false', the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from the side opposite the motor.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param name a name to associate with the motor for use in the motor menu
     * @param direction the direction that the motor spins in.
     * @param pulse the pulse frequency of the motor.
     */
    StepperMotor(
        Drivers* drivers,
        bool motorInverted,
        const char* name,
        tap::gpio::Digital::OutputPin direction,
        tap::gpio::Pwm::Pin pulse);

    void initialize() override;

    int64_t getEncoderUnwrapped() const override;

    uint16_t getEncoderWrapped() const override;

    /**
     * Sets the desired output for the motor.
     *
     * @param desiredOutput the desired motor output. Limited to the range of a 16-bit int.
     *
     * @note: `desiredOutput` is cast to an int16_t and limited to an int16_t's range! The
     *      user should make sure their value is in range. The declaration takes an int32_t
     *      in hopes to mitigate overflow.
     */
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
