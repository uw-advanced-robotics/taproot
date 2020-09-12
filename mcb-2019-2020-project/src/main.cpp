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

#include <aruwlib/rm-dev-board-a/board.hpp>

/* arch includes ------------------------------------------------------------*/
#include <aruwlib/architecture/periodic_timer.hpp>

/* communication includes ---------------------------------------------------*/
#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/display/sh1106.hpp>

/* error handling includes --------------------------------------------------*/

/* control includes ---------------------------------------------------------*/
#include "aruwsrc/control/robot_control.hpp"

using namespace modm::literals;
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
            drivers->errorController.update();
            drivers->commandScheduler.run();
            drivers->djiMotorTxHandler.processCanSendData();
        }
#ifndef ENV_SIMULATOR
        modm::delayMicroseconds(10);
#endif
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

#ifndef ENV_SIMULATOR
    /// \todo this should be an init in the display class
    Board::DisplaySpiMaster::
        connect<Board::DisplayMiso::Miso, Board::DisplayMosi::Mosi, Board::DisplaySck::Sck>();

    // SPI1 is on ABP2 which is at 90MHz; use prescaler 64 to get ~fastest baud rate below 1mHz max
    // 90MHz/64=~14MHz
    Board::DisplaySpiMaster::initialize<Board::SystemClock, 1406250_Hz>();
#endif
    aruwlib::display::Sh1106<
#ifndef ENV_SIMULATOR
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
#endif
        128,
        64,
        false>
        display;
    display.initializeBlocking();

    drivers->remote.initialize();
    drivers->mpu6500.init();
    drivers->refSerial.initialize();
    drivers->xavierSerial.initialize();
}

void updateIo(aruwlib::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->xavierSerial.updateSerial();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
}
