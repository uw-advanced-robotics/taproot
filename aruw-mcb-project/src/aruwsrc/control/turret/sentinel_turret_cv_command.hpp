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

#ifndef SENTINEL_TURRET_CV_COMMAND_HPP_
#define SENTINEL_TURRET_CV_COMMAND_HPP_

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/comprised_command.hpp"
#include "aruwlib/control/turret/turret_subsystem_interface.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/sentinel/firing/sentinel_rotate_agitator_command.hpp"
#include "aruwsrc/control/sentinel/firing/sentinel_switcher_subsystem.hpp"

namespace aruwsrc::control::turret
{
/**
 * A command that receives input from the vision system via the `XavierSerial` driver and aims the
 * turret accordingly.
 */
class SentinelTurretCVCommand : public aruwlib::control::ComprisedCommand
{
public:
    /**
     * Pitch/yaw error margins within which the auto aim deems it acceptable
     * to fire the launcher, in degrees.
     */
    static constexpr float YAW_FIRE_ERROR_MARGIN = 2.0f;
    static constexpr float PITCH_FIRE_ERROR_MARGIN = 2.0f;

    /**
     * Pitch/yaw angle increments that the turret will change by each call
     * to refresh when the turret is scanning for a target, in degrees.
     */
    static constexpr float SCAN_DELTA_ANGLE_YAW = 0.1f;
    static constexpr float SCAN_DELTA_ANGLE_PITCH = 0.1f;

    /**
     * The number of times refresh is called without receiving valid CV data to when
     * the command will consider the target lost and start tracking.
     */
    static constexpr int AIM_LOST_NUM_COUNTS = 500;

    SentinelTurretCVCommand(
        aruwlib::Drivers *drivers,
        aruwlib::control::turret::TurretSubsystemInterface *DoublePitchTurretSubsystem,
        aruwsrc::agitator::AgitatorSubsystem *agitatorSubsystem,
        sentinel::firing::SentinelSwitcherSubsystem *switcher);

    bool isReady() override { return sentinelTurret->isOnline(); }

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "sentinel turret cv"; }

    inline bool isAimingAtTarget() const { return aimingAtTarget; }

private:
    aruwlib::Drivers *drivers;

    aruwlib::control::turret::TurretSubsystemInterface *sentinelTurret;

    sentinel::firing::SentinelRotateAgitatorCommand rotateAgitator;

    bool pitchScanningUp;
    bool yawScanningRight;
    bool aimingAtTarget;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * CV no longer is tracking a target.
     */
    int lostTargetCounter;

    void scanForTarget();

    /**
     * Updates `axisScanningUp` based on the current setpoint and the min and max scanning
     * setpoints.
     */
    static void updateScanningUp(
        const float motorSetpoint,
        const float minMotorSetpoint,
        const float maxMotorSetpoint,
        bool *axisScanningUp);
};  // class SentinelTurretCVCommand

}  // namespace aruwsrc::control::turret

#endif  // SENTINEL_TURRET_CV_COMMAND_HPP_
