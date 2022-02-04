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

#include "modm/math/geometry/vector.hpp"

namespace tap::algorithms::ballistics
{

/**
 * Stores the 3D position, velocity, and acceleration of an object as `modm::Vector3f`s.
 * Position Units: m
 * Velocity Units: m/s
 * Acceleration Units: m/s^2
 */
struct MeasuredKinematicState
{
    modm::Vector3f position;    // m
    modm::Vector3f velocity;      // m/s
    modm::Vector3f acceleration;  // m/s^2
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
inline modm::Vector3f projectForward(const MeasuredKinematicState &state, float dt)
{
    return modm::Vector3f(
        quadraticKinematicProjection(
             dt,
             state.position.x,
             state.velocity.x,
             state.acceleration.x),
         quadraticKinematicProjection(
             dt,
             state.position.y,
             state.velocity.y,
             state.acceleration.y),
         quadraticKinematicProjection(
             dt,
             state.position.z,
             state.velocity.z,
             state.acceleration.z));
}

/**
 *
 * @param targetPosition: The 3D position of a target in m. Frame requirements: RELATIVE TO TURRET,
 * Z IS OPPOSITE TO GRAVITY.
 * @param bulletVelocity: The velocity of the projectile to be fired in m/s.
 *
 * @return The expected travel time of a turret shot to hit a target from this object's position.
 */
float computeTravelTime(
    const modm::Vector3f &targetPosition,
    float bulletVelocity);

/**
 * @param targetInitialState: The initial 3D kinematic state of a target. Frame requirements: RELATIVE TO TURRET,
 * Z IS OPPOSITE TO GRAVITY.
 * @param bulletVelocity: The velocity of the projectile to be fired in m/s.
 * @param numIterations: The number of times to project the kinematics forward (theoretically 1 is enough,
 * but more iterations could potentially reduce error).
 *
 * @return The position (in m, in the same frame as targetInitialState) at which our robot should aim to hit the given target,
 * taking into account the path a projectile takes to hit the target.
 */
modm::Vector3f findTargetProjectileIntersection(
    MeasuredKinematicState targetInitialState,
    float bulletVelocity,
    uint8_t numIterations);

/**
 * @param targetPosition: The position of the target relative to the turret as a 3D Vector.
 * 
 * @return The appropriate pitch angle in radians to hit the target, normalized in (-pi, pi].
 */
inline float computePitch(const modm::Vector3f &targetPosition)
{
    return atan2f(targetPosition.z, hypot(targetPosition.x, targetPosition.y));
}

/**
 * @param targetPosition: The position of the target relative to the turret as a 3D Vector.
 * 
 * @return The appropriate yaw angle in radians to hit the target, normalized in (-pi, pi].
 */
inline float computeYaw(const modm::Vector3f &targetPosition)
{
    return atan2f(targetPosition.y, targetPosition.x);
}

}  // namespace tap::algorithms::ballistics

#endif  // BALLISTICS_HPP_
