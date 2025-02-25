/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tap/drivers.hpp"

class Drivers : public tap::Drivers
{
public: 
    Drivers(): tap::Drivers() {}
};

Drivers drivers;

modm::Fiber<4096> io([]{
    while (true)
    {
        drivers.mpu6500.periodicIMUUpdate();
        modm::this_fiber::sleep_for(std::chrono::milliseconds(2));
    }
});

int main()
{
    Board::initialize();
    
    drivers.analog.init();
    drivers.pwm.init();
    drivers.digital.init();
    drivers.leds.init();
    drivers.can.initialize();
    drivers.errorController.init();
    drivers.remote.initialize();
    drivers.mpu6500.init(500.f, 0.1f, 0.f);
    drivers.refSerial.initialize();
    
    modm::fiber::Scheduler::run();
    return 0;
}
