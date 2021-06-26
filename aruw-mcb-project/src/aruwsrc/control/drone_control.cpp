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

#if defined(TARGET_DRONE)

#include "aruwlib/drivers_singleton.hpp"

using aruwlib::DoNotUse_getDrivers;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

namespace drone_control
{
/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() {}

/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems(aruwlib::Drivers *) {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands(aruwlib::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands(aruwlib::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings(aruwlib::Drivers *) {}
}  // namespace drone_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    drone_control::initializeSubsystems();
    drone_control::registerDroneSubsystems(drivers);
    drone_control::setDefaultDroneCommands(drivers);
    drone_control::startDroneCommands(drivers);
    drone_control::registerDroneIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
