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

#ifndef TURRET_MANUAL_COMMAND_H_
#define TURRET_MANUAL_COMMAND_H_

#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc
{
namespace turret
{
class TurretSubsystem;
/**
 * A command that controls the turret gimbals using a velocity PID controller.
 */
class TurretManualCommand : public aruwlib::control::Command
{
public:
    TurretManualCommand(aruwlib::Drivers *drivers, TurretSubsystem *subsystem);

    void initialize() override {}

    bool isFinished() const override;

    void execute() override;

    void end(bool) override {}

    const char *getName() const override { return "turret manual"; }

private:
    static constexpr float USER_INPUT_SCALAR = 50.0f;

    static constexpr float YAW_P = 1.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 0.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 16000;

    static constexpr float PITCH_P = 1.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 0.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 16000;

    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    float yawVelocityTarget = 0;
    float pitchVelocityTarget = 0;
};  // class TurretManualCommand

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_MANUAL_COMMAND_H_
