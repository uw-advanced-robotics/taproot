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

#ifdef PLATFORM_HOSTED
#include <iostream>
#endif

#include <aruwlib/rm-dev-board-a/board.hpp>
#include <modm/architecture/interface/delay.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/display/sh1106.hpp>

/* error handling includes --------------------------------------------------*/

/* control includes ---------------------------------------------------------*/
#include <aruwlib/architecture/clock.hpp>

#include "aruwsrc/control/robot_control.hpp"

using aruwlib::Drivers;

/* define timers here -------------------------------------------------------*/
aruwlib::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
void initializeIo(aruwlib::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
void updateIo(aruwlib::Drivers *drivers);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    aruwlib::Drivers *drivers = aruwlib::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    aruwsrc::control::initSubsystemCommands(drivers);

    while (1)
    {
        // do this as fast as you can
        updateIo(drivers);

        if (sendMotorTimeout.execute())
        {
            drivers->mpu6500.read();
            drivers->errorController.updateLedDisplay();
            drivers->commandScheduler.run();
            drivers->djiMotorTxHandler.processCanSendData();
            drivers->oledDisplay.updateMenu();
        }
        modm::delay_us(10);
    }
    return 0;
}

void initializeIo(aruwlib::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->mpu6500.init();
    drivers->refSerial.initialize();
    drivers->xavierSerial.initialize();
    drivers->oledDisplay.initialize();
}

void updateIo(aruwlib::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->xavierSerial.updateSerial();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->oledDisplay.updateDisplay();
}
