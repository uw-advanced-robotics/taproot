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

#ifndef AGITATOR_ROTATE_COMMAND_HPP_
#define AGITATOR_ROTATE_COMMAND_HPP_

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
 * pass in a rotate velocity and it uses `aruwlib::arch::getTimeMilliseconds()`
 * to determine the proper ramp increment.
 */
class AgitatorRotateCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] agitator The agitator associated with the rotate command.
     * @param[in] agitatorAngleChange The desired rotation angle, in radians.
     * @param[in] agitatorRotateTime The time it takes to rotate the agitator to the desired angle
     *      in milliseconds.
     * @param[in] agitatorPauseAfterRotateTime The time that the command will wait after rotating to
     *      the desired angle before the command is considered complete.
     * @param[in] setpointTolerance The angle difference between current and desired angle when the
     *      command will be considered to be completed (used in the `isFinished` function). Only set
     *      this if you want a different tolerance. `AGITATOR_SETPOINT_TOLERANCE` is usually fine.
     * @param[in] agitatorSetToFinalAngle `true` if you would like the agitator on `end` to set to
     *      the final desired rotation angle, `false` otherwise (in which case the final desired
     * angle set may be slightly shorter than the true desired angle change).
     * @attention the ramp value is calculated by finding the rotation speed
     *      (\f$agitatorAngleChange / agitatorRotateTime\f$), and then multiplying this by
     *      the period (how often the ramp is called)
     */
    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        uint32_t agitatorRotateTime,
        uint32_t agitatorPauseAfterRotateTime,
        bool agitatorSetToFinalAngle,
        float setpointTolerance = AGITATOR_SETPOINT_TOLERANCE);

    const char* getName() const override { return "agitator rotate"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    /**
     * The angle at which the agitator is considered to have reached its setpoint.
     */
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    AgitatorSubsystem* connectedAgitator;

    float agitatorTargetAngleChange;

    aruwlib::algorithms::Ramp rampToTargetAngle;

    /**
     * The time you want the agitator to take to rotate to the desired angle, in milliseconds.
     */
    uint32_t agitatorDesiredRotateTime;

    uint32_t agitatorMinRotatePeriod;

    aruwlib::arch::MilliTimeout agitatorMinRotateTimeout;

    float agitatorSetpointTolerance;

    uint32_t agitatorPrevRotateTime;

    bool agitatorSetToFinalAngle;
};  // class AgitatorRotateCommand

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_ROTATE_COMMAND_HPP_
