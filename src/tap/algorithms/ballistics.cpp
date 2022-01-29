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
    Ballistics::Ballistics(
        float bulletVelocity,
        modm::Vector<float, 3> initialTurretPosition,
        MeasuredKinematicState initialTargetState,
        bool targetDataValid) :
        bulletVelocity(bulletVelocity),
        turretPosition(initialTurretPosition),
        targetState(initialTargetState),
        targetDataValid(targetDataValid)
    {}

    float Ballistics::computeTravelTime(const modm::Vector<float, 3> &targetPosition) const
    {
        float horizontalDist = hypot(targetPosition[X] - turretPosition[X], targetPosition[Y] - turretPosition[Y]);
        float verticalDist = targetPosition[Z] - turretPosition[Z];
        float bulletVelocitySquared = powf(bulletVelocity, 2);
        float pitchAngle = atan((bulletVelocitySquared - sqrt(powf(bulletVelocitySquared, 2) - G*(G*powf(horizontalDist, 2) + 2*verticalDist*bulletVelocitySquared)))/(G*horizontalDist));

        return horizontalDist / (bulletVelocity * cos(pitchAngle));
    }

    modm::Vector<float, 3> Ballistics::findTargetProjectileIntersection()
    {
        modm::Vector<float, 3> projectedTargetPosition = targetState.position;
        float projectedTravelTime = 0;

        for (int i = 0; i < 5; i++)
        {
            projectedTravelTime = computeTravelTime(projectedTargetPosition);
            projectedTargetPosition = projectForward(targetState, projectedTravelTime);
        }

        return projectedTargetPosition;
    }

} // namespace algorithms

} // namespace tap
