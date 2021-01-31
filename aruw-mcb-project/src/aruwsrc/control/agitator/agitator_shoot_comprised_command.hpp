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

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_

#include <aruwlib/control/comprised_command.hpp>

#include "agitator_rotate_command.hpp"
#include "agitator_subsystem.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * A comprised command that combines the agitator unjam and rotate commands and provides
 * unjam monitoring to perform a single agitator rotation with unjamming if necessary.
 */
class ShootComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    /**
     * @param[in] agitator The agitator to interact with.
     * @param[in] agitatorChangeAngle The angle in radians that the agitator should rotate.
     * @param[in] maxUnjamAngle See `AgitatorUnJamCommand`'s constructor for more details,
     *      passed on directly to this command's constructor.
     * @param[in] agitatorDesiredRotateTime The desired time it takes to rotate, in milliseconds.
     * @param[in] minAgitatorRotateTime The minimum expected rotation time, in milliseconds.
     *      See `AgitatorRotateCommand`'s constructorfor more details.
     */
    ShootComprisedCommand(
        aruwlib::Drivers* drivers,
        AgitatorSubsystem* agitator,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        uint32_t agitatorDesiredRotateTime,
        uint32_t minAgitatorRotateTime);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator shoot"; }

private:
    AgitatorSubsystem* connectedAgitator;

    AgitatorRotateCommand agitatorRotateCommand;

    AgitatorUnjamCommand agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};  // class AgitatorShootComprisedCommand

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_HPP_
