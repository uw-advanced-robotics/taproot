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

namespace tap
{
namespace algorithms
{
    MeasuredKinematicState::MeasuredKinematicState(
        modm::Vector<float, 3> position,
        modm::Vector<float, 3> velocity,
        modm::Vector<float, 3> acceleration) :
        position(position),
        velocity(velocity),
        acceleration(acceleration)
    {}                   

    float MeasuredKinematicState::computeTravelTime(modm::Vector<float, 3> targetPosition)
    {
        float horizontalDist = hypot(targetPosition[0] - position[0], targetPosition[1] - position[1]);
        float verticalDist = targetPosition[2] - position[2];
        float pitchAngle = atan((powf(BULLET_VELOCITY, 2) - sqrt(powf(BULLET_VELOCITY, 4) - G*(G*powf(horizontalDist, 2) + 2*verticalDist*powf(BULLET_VELOCITY, 2))))/(G*horizontalDist));

        return horizontalDist / (BULLET_VELOCITY * cos(pitchAngle));
    }

    modm::Vector<float, 3> MeasuredKinematicState::findIntersection(MeasuredKinematicState targetInitialState)
    {
        modm::Vector<float, 3> projectedTargetPosition = targetInitialState.position;
        float projectedTravelTime = 0;

        for (int i = 0; i < 5; i++)
        {
            projectedTravelTime = computeTravelTime(projectedTargetPosition);
            projectedTargetPosition = projectForward(targetInitialState, projectedTravelTime);
        }

        return projectedTargetPosition;
    }

} // namespace algorithms

} // namespace tap
