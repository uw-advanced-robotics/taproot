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

#ifndef __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
class TurretWorldRelativePositionCommand : public aruwlib::control::Command
{
public:
    TurretWorldRelativePositionCommand(
        TurretSubsystem *subsystem,
        chassis::ChassisSubsystem *chassis);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret world relative position command"; }

private:
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 90.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;

    TurretSubsystem *turretSubsystem;
    chassis::ChassisSubsystem *chassisSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;

    float imuInitialYaw;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    void runYawPositionController();
    void runPitchPositionController();

    static float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle);
    static float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle);
};

}  // namespace turret

}  // namespace aruwsrc

#endif
