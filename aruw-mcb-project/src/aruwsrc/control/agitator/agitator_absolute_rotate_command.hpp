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

#ifndef AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_
#define AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
/**
 * A command that uses an agitator_subsystem to rotate to the same
 * angle everytime, attemping to rotate at the given angular velocity.
 * (Consistency doesn't work across motor disconnects). This command
 * ends immediately if the agitator is jammed, and upon ending will
 * stop the connected agitator by setting its target position to its
 * current position.
 *
 * Agitator angles are relative, and the "0"-angle is changed when
 * the agitator is calibrated.
 */
class AgitatorAbsoluteRotateCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] agitator the agitator subsystem this command depends on.
     * @param[in] targetAngle the target absolute angle relative the agitator
     *  should attempt to reach
     * @param[in] agitatorRotateSpeed The angular speed the agitator should
     *  attempt to move at in milliradians/second
     * @param[in] setpointTolerance the command will consider the target angle
     *  as reached when it's distance to the target is within this value
     */
    explicit AgitatorAbsoluteRotateCommand(
        aruwsrc::agitator::AgitatorSubsystem* agitator,
        float targetAngle,
        uint32_t agitatorRotateSpeed,
        float setpointTolerance);

    const char* getName() const override { return "open hopper lid"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

protected:
    aruwsrc::agitator::AgitatorSubsystem* connectedAgitator;

    /**
     * The angle at which the agitator is considered to have reached its setpoint.
     */
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = aruwlib::algorithms::PI / 16.0f;

    /**
     * Timeout to keep track of whether or not agitator has jammed. Reset to 0 whenever
     * execute is called and agitator is within AGITATOR_SETPOINT_TOLERANCE. Agitator
     * is considered jammed once this timeout reaches AGITATOR_JAM_TIMEOUT
     */
    aruwlib::arch::MilliTimeout jamTimeout;

private:
    /**
     * Max allowable period in milliseconds for agitator distance from target to be >=
     * AGITATOR_SETPOINT_TOLERANCE before agitator is considered jammed.
     */
    static constexpr uint32_t AGITATOR_JAM_TIMEOUT = 200;

    /* target angle for the agitator to reach when command is called.*/
    float targetAngle;

    aruwlib::algorithms::Ramp rampToTargetAngle;

    /**
     * The angular speed the agitator should attempt to move at in
     * milliradians/second.
     */
    uint32_t agitatorRotateSpeed;

    float agitatorSetpointTolerance;

    uint32_t agitatorPrevRotateTime;
};  // class AgitatorAbsoluteRotateCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // AGITATOR_ABSOLUTE_ROTATE_COMMAND_HPP_
