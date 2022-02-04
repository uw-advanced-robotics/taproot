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

#include "ballistics.hpp"
#include "math_user_utils.hpp"

namespace tap::algorithms::ballistics
{

float computeTravelTime(
    const modm::Vector3f &targetPosition,
    float bulletVelocity)
{
    float horizontalDist =
        hypot(targetPosition.x, targetPosition.y);
    float bulletVelocitySquared = powf(bulletVelocity, 2);
    float pitchAngle = atan2(
        (bulletVelocitySquared -
         sqrt(
             powf(bulletVelocitySquared, 2) -
             G * (G * powf(horizontalDist, 2) + 2 * targetPosition.z * bulletVelocitySquared))),
        (G * horizontalDist));

    return horizontalDist / (bulletVelocity * cos(pitchAngle));
}

modm::Vector3f findTargetProjectileIntersection(
    MeasuredKinematicState targetInitialState,
    float bulletVelocity,
    uint8_t numIterations)
{
    modm::Vector3f projectedTargetPosition = targetInitialState.position;
    float projectedTravelTime = 0;

    for (int i = 0; i < numIterations; i++)
    {
        projectedTravelTime =
            computeTravelTime(projectedTargetPosition, bulletVelocity);
        projectedTargetPosition = projectForward(targetInitialState, projectedTravelTime);
    }

    return projectedTargetPosition;
}

}  // namespace tap::algorithms::ballistics
