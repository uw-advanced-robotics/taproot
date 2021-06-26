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

#include "aruwlib/algorithms/ramp.hpp"
#include "aruwlib/control/command.hpp"
#include "aruwlib/control/turret/i_turret_subsystem.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that automatically rotates the chassis while maintaining turret angle
 */
class BeybladeCommand : public aruwlib::control::Command
{
public:
    BeybladeCommand(
        aruwlib::Drivers* drivers,
        ChassisSubsystem* chassis,
        const aruwlib::control::turret::iTurretSubsystem* turret);

    // fractional multiplier for user input and maximum motor speed to calculate desired x and y
    // speeds
    static constexpr float TRANSLATIONAL_SPEED_FRACTION_WHILE_BEYBLADE = 0.5f;

    // additional fractional multiplier/limit to calculate x and y translational speed limits
    // will be used to determine desired rotation speeds to ramp up to
    static constexpr float TRANSLATION_LIMITING_FRACTION = 0.5f;

    // set ramp targets for rotational speed + units unknown(?)
    /**
     * Use this rotation speed if power consumption limit is <= 45 W
     */
    static constexpr float ROTATION_TARGET_45W_CUTOFF = 3000.0f;
    /**
     * Use this rotation speed if power consumption limit is <= 60 W and > 45 W
     */
    static constexpr float ROTATION_TARGET_60W_CUTOFF = 3500.0f;
    /**
     * Use this rotation speed if power consumption limit is <= 80 W and > 60 W
     */
    static constexpr float ROTATION_TARGET_80W_CUTOFF = 4000.0f;
    /**
     * Use this rotation speed if power consumption limit is > 80 W
     */
    static constexpr float ROTATION_TARGET_MAX_CUTOFF = 4500.0f;
    /**
     * The fraction to cut rotation speed while moving and beyblading
     */
    static constexpr float RAMP_TARGET_TRANSLATIONAL_FRAC = 0.5f;
    /**
     * Fraction of the final setpoint to update the ramp target each time until
     * the final setpoint is reached
     */
    static constexpr float RAMP_UPDATE_FRAC = 0.125;

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
    float rotationDirection;

    aruwlib::algorithms::Ramp rotateSpeedRamp;

    aruwlib::Drivers* drivers;
    ChassisSubsystem* chassis;
    const aruwlib::control::turret::iTurretSubsystem* turret;

    float getRotationTarget() const;
};  // class BeybladeCommand

}  // namespace aruwsrc::chassis

#endif  // BEYBLADE_COMMAND_HPP_
