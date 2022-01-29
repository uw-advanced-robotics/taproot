/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALLISTICS_HPP_
#define BALLISTICS_HPP_

#include <cmath>
#include "modm/math.hpp"

namespace tap
{
namespace algorithms
{

enum Axes
    {
    X = 0,
    Y,
    Z,
};

/**
 * Stores the 3D position, velocity, and acceleration of an object.
 * Functionality for finding a position for our robot to aim at a given target
 * is also built into this class.
 */
struct MeasuredKinematicState
{
    modm::Vector<float, 3> position; // m
    modm::Vector<float, 3> velocity; // m/s
    modm::Vector<float, 3> acceleration; // m/s^2
};

class Ballistics {
public:
    /**
     * @param position The position of this object in 3D space, measured in meters.
     * @param velocity The velocity of this object in 3D space, measured in m/s.
     * @param acceleration The acceleration of this object in 3D space, measured in m/s^2.
     */
    Ballistics(
        float bulletVelocity,
        modm::Vector<float, 3> initialTurretPosition,
        MeasuredKinematicState initialTargetState,
        bool targetDataValid);

    /**
     * @param state: The kinematic state of the object to project forward.
     * @param dt: The amount of time to project the state forward.
     * 
     * @return The future 3D position of the objust using a quadratic (constant acceleration) model.
     */
    inline static modm::Vector<float, 3> projectForward(const MeasuredKinematicState &state, float dt)
    {
        return modm::Vector<float, 3>(
            {quadraticKinematicProjection(dt, state.position[X], state.velocity[X], state.acceleration[X]),
            quadraticKinematicProjection(dt, state.position[Y], state.velocity[Y], state.acceleration[Y]),
            quadraticKinematicProjection(dt, state.position[Z], state.velocity[Z], state.acceleration[Z])});
    }

    /**
     * The below states should be in the same coordinate frame in order for this method to work properly.
     * @param turretInitialState The inital state of the turret that will be firing.
     * @param targetInitialState The initial state of a target we want to fire at as a MeasuredKinematicState.
     * 
     * @return The position at which our robot should aim to hit the given target, taking into account the path a projectile takes to hit the target.
     */
    modm::Vector<float, 3> findTargetProjectileIntersection();

private:
    /**
     * @param dt: The amount of time to project forward.
     * @param s: The position of the object.
     * @param v: The velocity of the object.
     * @param a: The acceleration of the object.
     * 
     * @return The future position of an object using a quadratic (constant acceleration) model.
     */
    inline static float quadraticKinematicProjection(float dt, float s, float v, float a) {
        return s + v*dt + 0.5f*a*powf(dt, 2.0f);
    }

    /**
     * @return The expected travel time of a turret shot to hit a target from this object's position.
     */
    float computeTravelTime(const modm::Vector<float, 3> &targetPosition) const;

    modm::Vector<float, 3> turretPosition;
    MeasuredKinematicState targetState;
    bool targetDataValid = false;

    static constexpr float G = 9.81; // m/s^2
    float bulletVelocity; // m/s
};

}  // namespace algorithms

}  // namespace tap

#endif  // BALLISTICS_HPP_
