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

#ifndef MOTOR_INTERFACE_HPP_
#define MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace tap::motor
{
/**
 * An interface for interacting with a DC motor through a motor driver. The interface
 * is based off of the functions that DJI motors provide, and as such may not necessarily
 * be well suited to being implemented by other motors/motor controllers.
 *
 * The direction of positive rotation is implementation dependent. tap::motor:DjiMotor for example
 * allows either direction to be set as positive through a constructor parameter.
 */
class MotorInterface
{
public:
    /**
     * Initialize this motor. Performs any necessary setup to get the motor driver in working order.
     */
    virtual void initialize() = 0;

    /**
     * @return the unwrapped motor encoder value. Direction and angular representation are
     * implementation dependent.
     *
     * "Unwrapped" stems from how DJI motor encoders have a range of values which they loop through,
     * thus losing absolute position across rotations. The "unwrapped" motor encoder value maintains
     * an absolute position across rotations by accounting for full revolutions.
     *
     * This should return:
     *   num_revolutions * encoder_resolution + curr_encoder_value
     *
     * Where encoder_resolution is the number of ticks the encoder has before it loops and
     * num_resolutions is the
     */
    virtual int64_t getEncoderUnwrapped() const = 0;

    /**
     * @return the wrapped motor encoder value. Direction and angular representation are
     * implementation dependent.
     *
     * The wrapped encoder value is the "raw" encoder value in that it will always be restricted
     * to (wrapped around) the encoders bounds. Bounds are motor encoder dependent.
     */
    virtual uint16_t getEncoderWrapped() const = 0;

    /**
     * Set the desired output of this motor. The meaning of and valid range of this value is
     * motor dependent. The passed in value will be limited to whatever the min/max value
     * of the motor implementation is.
     *
     * The positive direction is implementation dependent.
     *
     * @param[in] desiredOutput the desired output
     */
    virtual void setDesiredOutput(int32_t desiredOutput) = 0;

    /**
     * @return `true` iff this motor is online. DjiMotor controllers will shutoff (stop broadcasting
     * over CAN) when they overheat for example and would be considered offline in that case.
     */
    virtual bool isMotorOnline() const = 0;

    /**
     * @return the last set desired output
     */
    virtual int16_t getOutputDesired() const = 0;

    /**
     * @return the temperature of this motor in degrees Celsius
     */
    virtual int8_t getTemperature() const = 0;

    /**
     * @return the current "Torque current" (DJI term; unsure what exactly this means... only the
     * current that goes towards driving motor?). In some units of Amps, (for C620 controller for
     * example units are (20/16384) Amps).
     */
    virtual int16_t getTorque() const = 0;

    /**
     * @return the current shaft angular velocity in RPM. Positive direction is implementation-
     * dependent
     */
    virtual int16_t getShaftRPM() const = 0;
};

}  // namespace tap::motor

#endif  //  MOTOR_INTERFACE_HPP_
