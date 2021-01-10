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

#ifndef XAXIS_SUBSYSTEM_HPP_
#define XAXIS_SUBSYSTEM_HPP_

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/subsystem.hpp>

namespace aruwsrc
{
namespace engineer
{
/**
 * This is a subsystem code for x-axis movement (moving the
 * grabber back and forward). Connect this to a digital output
 * pin. This controls a solenoid, which actuates a piston.
 */
class XAxisSubsystem : public aruwlib::control::Subsystem
{
public:
    XAxisSubsystem(aruwlib::Drivers *drivers, aruwlib::gpio::Digital::OutputPin pin)
        : aruwlib::control::Subsystem(drivers),
          pin(pin),
          extended(false)
    {
    }

    void setExtended(bool isExtended);

    bool isExtended() const;

    void runHardwareTests() override;

    const char *getName() override { return "X-Axis Subsystem"; }

private:
    aruwlib::gpio::Digital::OutputPin pin;

    bool extended;
};  // class XAxisSubsystem

}  // namespace engineer

}  // namespace aruwsrc

#endif  // XAXIS_SUBSYSTEM_HPP_
