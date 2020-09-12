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

#ifndef __AGITATOR_CALIBRATE_COMMAND_HPP__
#define __AGITATOR_CALIBRATE_COMMAND_HPP__

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/command.hpp>

#include "agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
/**
 * Default command that can be used to calibrate the agitator (i.e. spam calls
 * agitatorCalibrateHere). By default, the agitator will keep calling agitatorCalibrateHere
 * until the agitator is connected, however this command is for the following:
 *  - a placeholder command initially
 *  - allows you to recalibrate an agitator that has already been calibrated if necessary
 */
class AgitatorCalibrateCommand : public aruwlib::control::Command
{
public:
    explicit AgitatorCalibrateCommand(AgitatorSubsystem* agitator);

    const char* getName() const override { return "agitator calibrate command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    AgitatorSubsystem* agitator;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
