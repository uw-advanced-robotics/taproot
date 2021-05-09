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

#include "agitator_shoot_comprised_command_instances.hpp"

namespace aruwsrc
{
namespace agitator
{
static inline void initializeComprisedCommand(
    aruwlib::Drivers *drivers,
    const float heatLimitBuffer,
    const bool heatLimiting,
    aruwsrc::agitator::AgitatorRotateCommand *agitatorRotateCommand,
    aruwlib::control::CommandScheduler *comprisedCommandScheduler,
    bool *unjamSequenceCommencing)
{
    *unjamSequenceCommencing = false;
    const auto &robotData = drivers->refSerial.getRobotData();
    if (drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
        robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1)
    {
        return;
    }
    comprisedCommandScheduler->addCommand(agitatorRotateCommand);
}

void ShootFastComprisedCommand17MM::initialize()
{
    initializeComprisedCommand(
        drivers,
        HEAT_LIMIT_BUFFER,
        heatLimiting,
        &agitatorRotateCommand,
        &comprisedCommandScheduler,
        &unjamSequenceCommencing);
}

void ShootSlowComprisedCommand17MM::initialize()
{
    initializeComprisedCommand(
        drivers,
        HEAT_LIMIT_BUFFER,
        heatLimiting,
        &agitatorRotateCommand,
        &comprisedCommandScheduler,
        &unjamSequenceCommencing);
}
}  // namespace agitator
}  // namespace aruwsrc
