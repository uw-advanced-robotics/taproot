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

#ifndef TAPROOT_STEPPER_MOTOR_INTERFACE_HPP_
#define TAPROOT_STEPPER_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace tap::motor
{
/**
 * A generic stepper motor interface that controls the number
 * of steps a stepper motor takes. Positive number of steps
 * indicates counter-clockwise motion when looking at the shaft
 * from opposite the motor.
 * Origin position at startup is implementation defined, but can be calibrated
 * by calling calibrateOrigin().
 */
class StepperMotorInterface
{
public:
    /**
     * Commands the stepper motor to take the given number
     * of steps. Sign indicates direction.
     * @param numSteps is the number of steps to take.
     */
    inline void moveSteps(int numSteps) { desiredPosition += numSteps; };

    /**
     * Set the desired position of the stepper motor.
     * @note: the motor will attempt to move to this desired position.
     * @param desiredPosition is the desired position of the stepper motor.
     */
    inline void setDesiredPosition(int desiredPosition) { this->desiredPosition = desiredPosition; }

    /**
     * Returns the desired position.
     * @return the desired position.
     */
    inline int getDesiredPosition() const { return desiredPosition; }

    /**
     * Get the current position of the shaft measured in number of steps taken from the origin.
     * @return position.
     */
    inline int getCurrentPosition() const { return position; }

    /**
     * Calibrates origin such that the current position of the motor
     * is equal to currPos. More precisely, moves the origin to be -currPos from the
     * current physical position.
     * @param currPos current position of the motor.
     */
    inline void calibrateOrigin(int currPos) { position = currPos; }

    /**
     * Updates the stepper motor controller
     */
    virtual void refresh() = 0;

protected:
    // Prevents this class from being instantiated directly.
    StepperMotorInterface() = default;

    /**
     * Number of steps from the origin.
     */
    int position = 0;

    /**
     * Desired number of steps from origin.
     */
    int desiredPosition = 0;
};
}  // namespace tap::motor

#endif
