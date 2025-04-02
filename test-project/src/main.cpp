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
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "modm/architecture/interface/delay.hpp"

tap::Drivers drivers;
tap::encoder::CanEncoder encoder(&drivers, tap::encoder::CanEncoderId::ID3, tap::can::CanBus::CAN_BUS2);

int main()
{
    Board::initialize();


    drivers.can.initialize();
    encoder.initialize();

    while (1)
    {
        drivers.canRxHandler.pollCanData();
        modm::delay_us(10);
    }
    return 0;
}