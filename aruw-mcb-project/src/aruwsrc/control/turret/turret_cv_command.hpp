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

#ifndef TURRET_CV_COMMAND_HPP_
#define TURRET_CV_COMMAND_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace serial
{
class XavierSerial;
}
namespace turret
{
/**
 * A command that receives input from the vision system via the `XavierSerial` driver and aims the
 * turret accordingly using a position PID controller.
 */
class TurretCVCommand : public aruwlib::control::Command
{
public:
    TurretCVCommand(aruwlib::Drivers *xavierSerial, TurretSubsystem *subsystem);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret cv"; }

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

    static constexpr float PITCH_P = 3500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 80.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    uint32_t prevTime;

    void runYawPositionController(float dt);

    void runPitchPositionController(float dt);
};  // class TurretCvCommand

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_CV_COMMAND_HPP_
