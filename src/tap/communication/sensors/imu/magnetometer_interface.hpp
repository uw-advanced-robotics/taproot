/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_MAGNETOMETER_INTERFACE_HPP_
#define TAPROOT_MAGNETOMETER_INTERFACE_HPP_

#include "modm/math/geometry.hpp"

namespace tap::communication::sensors::imu
{
/**
 * An interface for interacting with a 3 axis magnetometer.
 */
class MagnetometerInterface
{
public:
    /**
     * Possible magnetometer states for a magnetometer.
     */
    enum class MagnetometerState
    {
        /** Indicates the magnetometer's init function was not called or initialization failed, so
           data from this class will be undefined. */
        MAGNETOMETER_NOT_CONNECTED,
        /** Indicates the magnetometer is connected and reading data, but calibration ranges have
           not been computed. */
        MAGNETOMETER_NOT_CALIBRATED,
        /** Indicates the magnetometer is in the process of computing calibration offsets. Data read
           when the magnetometer is in this state is undefined. */
        MAGNETOMETER_CALIBRATING,
        /// Indicates the magnetometer is connected and calibration offsets have been computed.
        MAGNETOMETER_CALIBRATED,
    };

    /**
     * Returns the magnetic field heading
     */
    virtual inline float getHeading() = 0;

    /**
     * Returns the magnetometer's state.
     */
    virtual inline MagnetometerState getState() const = 0;

    /**
     * Begins calibration process for the magnetometer.
     */
    virtual inline void requestCalibration() = 0;
};

}  // namespace tap::communication::sensors::imu

#endif
