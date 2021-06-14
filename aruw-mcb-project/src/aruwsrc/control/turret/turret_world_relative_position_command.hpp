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

#ifndef TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_
#define TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
class ChassisSubsystem;
}

namespace turret
{
class TurretSubsystem;

/**
 * Turret control, with the yaw gimbal using the world relative frame, such that the
 * desired turret angle is independent of the direction that the chassis is facing
 * or rotating. Assumes the Mpu6500 used for calculations is mounted on the chassis.
 */
class TurretWorldRelativePositionCommand : public aruwlib::control::Command
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     * The `ChassisSubsystem` is only used for for odometry information.
     */
    TurretWorldRelativePositionCommand(
        aruwlib::Drivers *drivers,
        TurretSubsystem *subsystem,
        const chassis::ChassisSubsystem *chassis,
        bool useImuOnTurret = false);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret world relative position"; }

private:
    static constexpr float YAW_P = 3800.0f;
    static constexpr float YAW_I = 50.0f;
    static constexpr float YAW_D_TURRET_IMU = 4300.0f;
    static constexpr float YAW_D_CHASSIS_IMU = 180.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 1000.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 10.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 10.0f;

    static constexpr float PITCH_P = 3200.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 120.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 47.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 1.5f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.6f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;

    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;
    const chassis::ChassisSubsystem *chassisSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;

    float imuInitialYaw;

    uint32_t prevTime;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    const bool useImuOnTurret;
    bool usingImuOnTurret;

    int blinkCounter = 0;

    void runYawPositionController(float dt);
    void runPitchPositionController(float dt);

    float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle);
    float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle);
};  // class TurretWorldRelativePositionCommand

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP_
