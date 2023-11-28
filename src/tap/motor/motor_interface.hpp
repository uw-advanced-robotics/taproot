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

#ifndef TAPROOT_MOTOR_INTERFACE_HPP_
#define TAPROOT_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace tap::motor
{
class MotorInterface
{
public:
    /**
     * Initialize the motor.
     *
     * @warning Must be called before any other methods are called. There are no checks that ensure
     * this.
     */
    virtual void initialize() = 0;

    /**
     * Set the desired output for the motor. The meaning of this value is motor
     * controller specific.
     *
     * @param[in] desiredOutput the desired motor output. Limited to the range of a 16-bit int.
     *
     * @note: `desiredOutput` is cast to an int16_t and limited to an int16_t's range! The
     *      user should make sure their value is in range. The declaration takes an int32_t
     *      in hopes to mitigate overflow.
     */
    virtual void setDesiredOutput(int32_t desiredOutput) = 0;

    /**
     * @return the current output value to be sent to the motor controller
     *      (the value last passed into @see setDesiredOutput())
     */
    virtual int16_t getOutputDesired() const = 0;

    /**
     * Gets the unwrapped number of encoder ticks in the "forward" motor direction.
     */
    virtual int64_t getEncoderUnwrapped() const = 0;

    /**
     * Gets the wrapped number of encoder ticks in the "forward" motor direction.
     */
    virtual uint16_t getEncoderWrapped() const = 0;

    /**
     * Forcefully offsets the motor's number of encoder revolutions.
     */
    virtual void offsetRevolutions(int64_t revolutionsOffset) = 0;

    /**
     * Resets the current motor position to have encoder value 0 unwrapped.
     */
    virtual void resetEncoderValue() = 0;

    /**
     * Gets the unwrapped angle of the motor in the "forward" direction.
     */
    virtual float getPositionUnwrapped() const = 0;

    /**
     * Gets the wrapped angle of the motor in the "forward" direction.
     */
    virtual float getPositionWrapped() const = 0;

    /**
     * @return whether motor is still reachable.
     */
    virtual bool isMotorOnline() const = 0;

    /**
     * @return the temperature of the motor as reported by the motor in degrees Celsius
     */
    virtual int8_t getTemperature() const = 0;

    /**
     * @return the current torque exerted by the motor.
     */
    virtual int16_t getTorque() const = 0;

    /**
     * @return the motor RPM.
     */
    virtual int16_t getShaftRPM() const = 0;
};

}  // namespace tap::motor

#endif  // TAPROOT_MOTOR_INTERFACE_HPP_
