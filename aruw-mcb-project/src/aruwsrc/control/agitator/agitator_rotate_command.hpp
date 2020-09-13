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

#ifndef __AGITATOR_ROTATE_COMMAND_HPP__
#define __AGITATOR_ROTATE_COMMAND_HPP__

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Rotates the connected agitator some angle in some desired time. Currently
 * pass in a rotate velocity and it uses modm::Clock::now() to determine the
 * proper ramp increment.
 */
class AgitatorRotateCommand : public aruwlib::control::Command
{
private:
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    AgitatorSubsystem* connectedAgitator;

    float agitatorTargetAngleChange;

    aruwlib::algorithms::Ramp rampToTargetAngle;

    // time you want the agitator to take to rotate to the desired angle, in milliseconds
    uint32_t agitatorDesiredRotateTime;

    uint32_t agitatorMinRotatePeriod;

    aruwlib::arch::MilliTimeout agitatorMinRotateTimeout;

    float agitatorSetpointTolerance;

    uint32_t agitatorPrevRotateTime;

    bool agitatorSetToFinalAngle;

public:
    /**
     * @param agitator the agitator associated with the rotate command
     * @param agitatorAngleChange the desired rotation angle
     * @param agitatorRotateTime the time it takes to rotate the agitator to the desired angle
     *                           in milliseconds
     * @param setpointTolerance the angle difference between current and desired angle when the
     *                          command will be considered to be completed (used in isFinished
     *                          function). Only set this if you want a different tolerance,
     *                          otherwise the above tolerance is usually fine.
     * @attention the ramp value is calculated by finding the rotation speed
     *            (agitatorAngleChange / agitatorRotateTime), and then multiplying this by
     *            the period (how often the ramp is called)
     */
    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        uint32_t agitatorRotateTime,
        uint32_t agitatorPauseAfterRotateTime,
        bool agitatorSetToFinalAngle,
        float setpointTolerance = AGITATOR_SETPOINT_TOLERANCE);

    const char* getName() const override { return "agitator rotate command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
