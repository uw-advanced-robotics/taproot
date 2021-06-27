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

#ifndef FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_
#define FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_

#include "aruwlib/control/command.hpp"

#include "friction_wheel_subsystem.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc::control::launcher
{
/**
 * Rotates flywheels of 17MM projectile launcher based on ref serial speed limit.
 */
class FrictionWheelSpinRefLimitedCommand : public aruwlib::control::Command
{
public:
    FrictionWheelSpinRefLimitedCommand(
        aruwlib::Drivers *drivers,
        aruwsrc::launcher::FrictionWheelSubsystem *frictionWheels);

    void initialize() override {}

    void execute() override;

    void end(bool) override { frictionWheels->setDesiredRpm(0); }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "ref serial friction wheel rotate"; }

private:
    static constexpr int16_t WHEEL_RPM_15 = 4500;
    static constexpr int16_t WHEEL_RPM_18 = 5000;
    static constexpr int16_t WHEEL_RPM_30 = 7000;

    aruwlib::Drivers *drivers;

    aruwsrc::launcher::FrictionWheelSubsystem *frictionWheels;
};
}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_
