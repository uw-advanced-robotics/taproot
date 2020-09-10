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

#ifndef __AGITATOR_UNJAM_COMMAND_HPP__
#define __AGITATOR_UNJAM_COMMAND_HPP__

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
class AgitatorUnjamCommand : public aruwlib::control::Command
{
public:
    AgitatorUnjamCommand(
        AgitatorSubsystem* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator unjam command"; }

private:
    static constexpr uint32_t SALVATION_TIMEOUT_MS = 2000;

    static constexpr uint32_t SALVATION_UNJAM_BACK_WAIT_TIME = 1000;

    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    // the maximum time that the command will wait from commanding the agitator to rotate
    // backwards to rotating forwards again.
    static constexpr uint32_t AGITATOR_MAX_WAIT_TIME = 130;

    // minimum angle the agitator will rotate backwards when unjamming
    static constexpr float MIN_AGITATOR_UNJAM_ANGLE = aruwlib::algorithms::PI / 4.0f;

    enum AgitatorUnjamState
    {
        AGITATOR_SALVATION_UNJAM_BACK,
        AGITATOR_UNJAM_BACK,
        AGITATOR_UNJAM_RESET,
        FINISHED
    };

    AgitatorUnjamState currUnjamstate;

    // time allowed to rotate back the the currAgitatorUnjamAngle
    aruwlib::arch::MilliTimeout agitatorUnjamRotateTimeout;

    aruwlib::arch::MilliTimeout salvationTimeout;

    // usually set to AGITATOR_MAX_WAIT_TIME, but can be user enabled
    uint32_t agitatorMaxWaitTime;

    AgitatorSubsystem* connectedAgitator;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
