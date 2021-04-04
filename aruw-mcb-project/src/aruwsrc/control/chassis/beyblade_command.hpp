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

#ifndef BEYBLADE_COMMAND_HPP_
#define BEYBLADE_COMMAND_HPP_

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwlib/algorithms/ramp.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class BeybladeCommand : public aruwlib::control::Command
{
public:
    BeybladeCommand(
        aruwlib::Drivers* drivers,
        ChassisSubsystem* chassis,
        aruwsrc::turret::TurretSubsystem* turret)
        : drivers(drivers),
          chassis(chassis),
          turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(chassis));
    }

    // fractional multiplier for user input and maximum motor speed to calculate desired x and y
    // speeds
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_BEYBLADE = 0.5f;

    // additional fractional multiplier/limit to calculate x and y translational speed limits
    // will be used to determine desired rotation speeds to ramp up to
    static constexpr float TRANSLATION_LIMITING_FRACTION = 0.5f;

    // set ramp targets for rotational speed + units unknown(?)
    static constexpr float RAMP_TARGET_NON_TRANSLATIONAL = 7000;
    static constexpr float RAMP_TARGET_TRANSLATIONAL = 3500;

    /**
     * Sets rotational input target on Ramp
     */
    void initialize() override;

    /**
     * Updates rotational speed with ramp, translates and rotates x and y inputs based on turret
     * angle Sets chassis motor outputs
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis beyblade"; }

private:
    float rampTarget;
    static constexpr float rampUpdate = 0.125;

    aruwlib::algorithms::Ramp rotateSpeedRamp;

    aruwlib::Drivers* drivers;
    ChassisSubsystem* chassis;
    aruwsrc::turret::TurretSubsystem* turret;

};  // class BeybladeCommand

}  // namespace chassis

}  // namespace aruwsrc

#endif  // BEYBLADE_COMMAND_HPP_
