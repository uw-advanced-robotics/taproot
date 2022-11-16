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

#ifndef TAPROOT_EXTERNAL_POWER_SOURCE_INTERFACE_HPP_
#define TAPROOT_EXTERNAL_POWER_SOURCE_INTERFACE_HPP_

namespace tap::communication::sensors::power
{
/**
 * Interface for generic external power source on the robot, such as a capacitor bank.
 */
class ExternalPowerSourceInterface
{
public:
    /**
     * Returns the amount of power stored in the external source
     */
    virtual int getAvailablePower() const = 0;

    /**
     * Removes power from the external source. Returns either 0 or the amount of power not able to
     * be removed from the external source
     */
    virtual int consumeAvailablePower(int consumed) = 0;
};
}  // namespace tap::communication::sensors::power

#endif  // TAPROOT_EXTERNAL_POWER_SOURCE_INTERFACE_HPP_
