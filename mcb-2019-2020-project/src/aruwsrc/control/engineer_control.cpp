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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/command_scheduler.hpp>

#include "aruwsrc/control/engineer/extend_xaxis_command.hpp"
#include "aruwsrc/control/engineer/grabber_subsystem.hpp"
#include "aruwsrc/control/engineer/squeeze_grabber_command.hpp"
#include "aruwsrc/control/engineer/xaxis_subsystem.hpp"

#if defined(TARGET_ENGINEER)

using namespace aruwsrc::engineer;
using namespace aruwlib::gpio;
using aruwlib::Drivers;
using aruwlib::control::CommandMapper;

namespace aruwsrc
{
namespace control
{
const Digital::OutputPin grabberPin = Digital::OutputPin::E;
const Digital::OutputPin xAxisPin = Digital::OutputPin::F;

/* define subsystems --------------------------------------------------------*/
GrabberSubsystem grabber(grabberPin);
XAxisSubsystem xAxis(xAxisPin);

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems()
{
    Drivers::commandScheduler.registerSubsystem(&grabber);
    Drivers::commandScheduler.registerSubsystem(&xAxis);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands() {}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands() {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings() {}

void initSubsystemCommands()
{
    registerEngineerSubsystems();
    setDefaultEngineerCommands();
    startEngineerCommands();
    registerEngineerIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
