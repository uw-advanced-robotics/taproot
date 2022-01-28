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
        modm::Vector<float, 3> acceleration,
        float bulletVelocity) :
        position(position),
        velocity(velocity),
        acceleration(acceleration),
        bulletVelocity(bulletVelocity)
    {}

    float MeasuredKinematicState::computeTravelTime(const modm::Vector<float, 3> &targetPosition) const
    {
        float horizontalDist = hypot(targetPosition[X] - position[X], targetPosition[Y] - position[Y]);
        float verticalDist = targetPosition[Z] - position[Z];
        float bulletVelocitySquared = powf(bulletVelocity, 2);
        float pitchAngle = atan((bulletVelocitySquared - sqrt(powf(bulletVelocitySquared, 2) - G*(G*powf(horizontalDist, 2) + 2*verticalDist*bulletVelocitySquared)))/(G*horizontalDist));

        return horizontalDist / (bulletVelocity * cos(pitchAngle));
    }

    static modm::Vector<float, 3> findTargetProjectileIntersection(const MeasuredKinematicState &turretState, const MeasuredKinematicState &targetInitialState)
    {
        modm::Vector<float, 3> projectedTargetPosition = targetInitialState.position;
        float projectedTravelTime = 0;

        for (int i = 0; i < 5; i++)
        {
            projectedTravelTime = turretState.computeTravelTime(projectedTargetPosition);
            projectedTargetPosition = targetInitialState.projectForward(projectedTravelTime);
        }

        return projectedTargetPosition;
    }

} // namespace algorithms

} // namespace tap
