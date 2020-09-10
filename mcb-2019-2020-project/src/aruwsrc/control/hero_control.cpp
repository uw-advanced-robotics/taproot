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

#if defined(TARGET_HERO)

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/

/* define commands ----------------------------------------------------------*/

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems() {}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands() {}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands() {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings() {}

void initSubsystemCommands()
{
    registerHeroSubsystems();
    setDefaultHeroCommands();
    startHeroCommands();
    registerHeroIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
