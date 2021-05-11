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

#ifndef HOPPER_COMMANDS_HPP_
#define HOPPER_COMMANDS_HPP_

#include "aruwlib/algorithms/math_user_utils.hpp"

#include "aruwsrc/control/agitator/agitator_absolute_rotate_command.hpp"

namespace aruwsrc
{
namespace control
{
/**
 * For opening and closing the hopper lid currently we use overly large
 * target angles for both opening and closing, this way (assuming no
 * interruption) the lid will keep moving until it hits the end of its
 * range of motion and "jams". Then we tell the motor to stop trying to
 * move and set it there. With this approach regardless of disconnects
 * and recalibrations we can hope to utilize the full hopper range of motion.
 */
class SoldierOpenHopperCommand : public AgitatorAbsoluteRotateCommand
{
public:
    // 30 revolutions
    static constexpr float SOLDIER_OPEN_HOPPER_TARGET_ANGLE = 30.0f * aruwlib::algorithms::PI;
    // 5000 milliradians/second
    static constexpr uint32_t SOLDIER_OPEN_HOPPER_ANGULAR_SPEED =
        5.0f * aruwlib::algorithms::PI * 1000.0f;
    // Allowable error in radians within which motor will consider target angle reached.
    static constexpr float SOLDIER_OPEN_HOPPER_TOLERANCE = 0.125f;
    SoldierOpenHopperCommand(agitator::AgitatorSubsystem* agitator)
        : AgitatorAbsoluteRotateCommand(
              agitator,
              SOLDIER_OPEN_HOPPER_TARGET_ANGLE,
              SOLDIER_OPEN_HOPPER_ANGULAR_SPEED,
              SOLDIER_OPEN_HOPPER_TOLERANCE)
    {
    }
};  // class SoldierOpenHopperCommand

/**
 * Use overly large target angle to keep moving agitator in one direction until
 * it "jams" and then stop moving the motor.
 *
 * This command is default scheduled so it has a slight twist, so as to not
 * overheat motors once it "jams" (which we use for detecting end of range
 * of motion) we set desired position to current position and continuously
 * return `false` from isFinished.
 */
class SoldierCloseHopperCommand : public AgitatorAbsoluteRotateCommand
{
public:
    // -30 revolutions
    static constexpr float SOLDIER_CLOSE_HOPPER_TARGET_ANGLE = -30.0f * aruwlib::algorithms::PI;
    // 5000 milliradians/second
    static constexpr uint32_t SOLDIER_CLOSE_HOPPER_ANGULAR_SPEED =
        5.0f * aruwlib::algorithms::PI * 1000.0f;
    // Allowable error in radians within which motor will consider target angle reached.
    static constexpr float SOLDIER_CLOSE_HOPPER_TOLERANCE = 0.125f;

    SoldierCloseHopperCommand(agitator::AgitatorSubsystem* agitator)
        : AgitatorAbsoluteRotateCommand(
              agitator,
              SOLDIER_CLOSE_HOPPER_TARGET_ANGLE,
              SOLDIER_CLOSE_HOPPER_ANGULAR_SPEED,
              SOLDIER_CLOSE_HOPPER_TOLERANCE)
    {
    }

    bool isFinished() const override
    {
        if (jamTimeout.isExpired())
        {
            float currAngle = connectedAgitator->getAgitatorAngle();
            connectedAgitator->setAgitatorDesiredAngle(currAngle);
        }
        return false;
    }
};  // class SoldierCloseHopperCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // HOPPER_COMMANDS_HPP_
