/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "friction_wheel_subsystem.hpp"

#include <aruwlib/architecture/clock.hpp>

namespace aruwsrc
{
namespace launcher
{
void FrictionWheelSubsystem::setDesiredRpm(float desRpm) { desiredRpmRamp.setTarget(desRpm); }

void FrictionWheelSubsystem::refresh()
{
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
    prevTime = currTime;

    velocityPidLeftWheel.update(desiredRpmRamp.getValue() - leftWheel.getShaftRPM());
    leftWheel.setDesiredOutput(static_cast<int32_t>(velocityPidLeftWheel.getValue()));
    velocityPidRightWheel.update(desiredRpmRamp.getValue() - rightWheel.getShaftRPM());
    rightWheel.setDesiredOutput(static_cast<int32_t>(velocityPidRightWheel.getValue()));
}
}  // namespace launcher

}  // namespace aruwsrc
