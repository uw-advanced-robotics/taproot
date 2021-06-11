/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef I_CHASSIS_SUBSYSTEM_HPP_
#define I_CHASSIS_SUBSYSTEM_HPP_

#include "aruwlib/motor/dji_motor.hpp"

#include "../subsystem.hpp"

namespace aruwlib::control::chassis
{
class IChassisSubsystem : public Subsystem
{
public:
    IChassisSubsystem(Drivers *drivers) : Subsystem(drivers) {}

    /**
     * @return the number of chassis motors
     */
    virtual inline int getNumChassisMotors() const = 0;

    virtual inline int16_t getLeftFrontRpmActual() const = 0;
    virtual inline int16_t getLeftBackRpmActual() const = 0;
    virtual inline int16_t getRightFrontRpmActual() const = 0;
    virtual inline int16_t getRightBackRpmActual() const = 0;
};
}  // namespace aruwlib::control::chassis

#endif  // I_CHASSIS_SUBSYSTEM_HPP_
