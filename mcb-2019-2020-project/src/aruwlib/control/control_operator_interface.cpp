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

#include "control_operator_interface.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"

using namespace aruwlib;
using namespace aruwlib::algorithms;

namespace aruwlib
{
namespace control
{
float ControlOperatorInterface::getChassisXInput()
{
    if (prevUpdateCounterX != drivers->remote.getUpdateCounter())
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL));
    }
    prevUpdateCounterX = drivers->remote.getUpdateCounter();
    return aruwlib::algorithms::limitVal<float>(
        chassisXInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::W)) -
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::S)),
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getChassisYInput()
{
    if (prevUpdateCounterY != drivers->remote.getUpdateCounter())
    {
        chassisYInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL));
    }
    prevUpdateCounterY = drivers->remote.getUpdateCounter();
    return aruwlib::algorithms::limitVal<float>(
        chassisYInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::A)) -
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::D)),
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getChassisRInput()
{
    if (prevUpdateCounterZ != drivers->remote.getUpdateCounter())
    {
        chassisRInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL));
    }
    prevUpdateCounterZ = drivers->remote.getUpdateCounter();
    return aruwlib::algorithms::limitVal<float>(
        chassisRInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::Q)) -
            static_cast<float>(drivers->remote.keyPressed(Remote::Key::E)),
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return -static_cast<float>(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL)) +
           static_cast<float>(drivers->remote.getMouseX()) * USER_MOUSE_YAW_SCALAR;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    return static_cast<float>(drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL)) +
           static_cast<float>(drivers->remote.getMouseY()) * USER_MOUSE_PITCH_SCALAR;
}

float ControlOperatorInterface::getSentinelSpeedInput()
{
    return drivers->remote.getChannel(aruwlib::Remote::Channel::LEFT_HORIZONTAL) *
           USER_STICK_SENTINEL_DRIVE_SCALAR;
}
}  // namespace control

}  // namespace aruwlib
