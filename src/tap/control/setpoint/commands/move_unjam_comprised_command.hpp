/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"

#include "move_command.hpp"
#include "unjam_command.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
// Forward declarations
class SetpointSubsystem;

/**
 * A comprised command that combines the unjam and move commands and provides
 * unjam monitoring to perform a single agitator rotation with unjamming if necessary.
 */
class MoveUnjamComprisedCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @param[in] drivers A pointer to the `Drivers` struct
     * @param[in] setpointSubsystem The subsystem to interact with.
     * @param[in] moveDisplacement The displacement the command will apply to the subsystem
     * @param[in] unjamDisplacement See `UnjamCommand`'s constructor for more details,
     *      passed on directly to this command's constructor.
     * @param[in] unjamThreshold See `UnjamCommand`'s constructor for more details,
     *      passed on directly to this command's constructor.
     * @param[in] maxUnjamWaitTime See `UnjamCommand`'s constructor for more details,
     *      passed on directly to this command's constructor.
     * @param[in] moveTime The time it takes to rotate the agitator to the desired angle
     *      in milliseconds.
     * @param[in] pauseAfterMoveTime See `MoveCommand` for more details, passed on directly
     *      to its constructor.
     * @param[in] setToTargetOnEnd See `MoveCommand` for more details, passed on directly
     *      to its constructor.
     * @param[in] setpointTolerance See `MoveCommand` for more details, passed on directly
     *      to its constructor.
     */
    MoveUnjamComprisedCommand(
        tap::Drivers* drivers,
        SetpointSubsystem* setpointSubsystem,
        float moveDisplacement,
        float unjamDisplacement,
        float unjamThreshold,
        uint32_t maxUnjamWaitTime,
        uint32_t moveTime,
        uint32_t pauseAfterMoveTime,
        bool setToTargetOnEnd,
        float setpointTolerance);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator shoot"; }

protected:
    SetpointSubsystem* setpointSubsystem;

    MoveCommand agitatorRotateCommand;

    UnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;

    bool agitatorDisconnectFault;
};  // class MoveUnjamComprisedCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
