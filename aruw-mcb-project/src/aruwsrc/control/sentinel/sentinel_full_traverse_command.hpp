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

#if defined(TARGET_SENTINEL)
#ifndef SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
#define SENTINEL_FULL_TRAVERSE_COMMAND_HPP_

#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/control/command.hpp>

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class SentinelDriveSubsystem;

class SentinelFullTraverseCommand : public aruwlib::control::Command
{
public:
    explicit SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem);

    const char* getName() const override { return "sentinel full traverse"; }
    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;

private:
    /**
     * Rate of change of the sentinel when changing direction, in wheel RPM / ms
     */
    static constexpr float RAMP_SPEED = 10.0f;

    /**
     * The rotational speed of the sentinel's wheels before gearing is applied, in RPM.
     */
    static constexpr float MAX_DESIRED_TRAVERSE_SPEED = 4000.0f;

    /**
     * The distance from the end of the rail at which the sentinel will referse direction.
     */
    static constexpr float TURNAROUND_BUFFER = 0.2f * SentinelDriveSubsystem::RAIL_LENGTH;

    uint32_t prevTime;

    aruwlib::algorithms::Ramp velocityTargetGenerator;

    SentinelDriveSubsystem* subsystemSentinelDrive;
};  // class SentinelFullTraverseCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
#endif
