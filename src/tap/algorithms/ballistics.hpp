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

namespace tap::algorithms::ballistics
{

enum Axes
{
    X = 0,
    Y,
    Z,
};

/**
 * Stores the 3D position, velocity, and acceleration of an object as modm::Vectors.
 * Position Units: m
 * Velocity Units: m/s
 * Acceleration Units: m/s^2
 */
struct MeasuredKinematicState
{
    modm::Vector<float, 3> position;      // m
    modm::Vector<float, 3> velocity;      // m/s
    modm::Vector<float, 3> acceleration;  // m/s^2
};

/**
 * @param dt: The amount of time to project forward.
 * @param s: The position of the object.
 * @param v: The velocity of the object.
 * @param a: The acceleration of the object.
 *
 * @return The future position of an object using a quadratic (constant acceleration) model.
 */
inline float quadraticKinematicProjection(float dt, float s, float v, float a)
{
    return s + v * dt + 0.5f * a * powf(dt, 2.0f);
}

/**
 * @param state: The kinematic state of the object to project forward.
 * @param dt: The amount of time to project the state forward.
 *
 * @return The future 3D position of the object using a quadratic (constant acceleration) model.
 */
inline modm::Vector<float, 3> projectForward(const MeasuredKinematicState &state, float dt)
{
    return modm::Vector<float, 3>(
        {quadraticKinematicProjection(
             dt,
             state.position[X],
             state.velocity[X],
             state.acceleration[X]),
         quadraticKinematicProjection(
             dt,
             state.position[Y],
             state.velocity[Y],
             state.acceleration[Y]),
         quadraticKinematicProjection(
             dt,
             state.position[Z],
             state.velocity[Z],
             state.acceleration[Z])});
}

/**
 * The below positions should be in the same xyz coordinate frame in order for this method to work
 * properly. In this coordinate system, Z MUST BE DEFINED AS OPPOSITE TO GRAVITY.
 * 
 * @param turretPosition: The 3D position of the turret in m.
 * @param targetPosition: The 3D position of the target to be fired at in m.
 * @param bulletVelocity: The velocity of the projectile to be fired in m/s.
 * 
 * @return The expected travel time of a turret shot to hit a target from this object's position.
 */
float computeTravelTime(const modm::Vector<float, 3> &turretPosition, const modm::Vector<float, 3> &targetPosition, float bulletVelocity);

/**
 * The below states should be in the same xyz coordinate frame in order for this method to work
 * properly. In this coordinate system, Z MUST BE DEFINED AS OPPOSITE TO GRAVITY.
 * 
 * @param turretPosition: The current 3D position of the turret that will be firing in m.
 * @param targetInitialState: The initial 3D kinematic state of a target we want to fire at.
 * @param bulletVelocity: The velocity of the projectile to be fired in m/s.
 *
 * @return The position at which our robot should aim to hit the given target, taking into account
 * the path a projectile takes to hit the target.
 */
modm::Vector<float, 3> findTargetProjectileIntersection(modm::Vector<float, 3> turretPosition, MeasuredKinematicState targetInitialState, float bulletVelocity);

static constexpr float G = 9.81;  // m/s^2

}  // namespace tap::algorithms::ballistics

#endif  // BALLISTICS_HPP_
