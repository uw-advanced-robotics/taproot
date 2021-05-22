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

#ifndef CAN_RX_LISTENER_MOCK_HPP_
#define CAN_RX_LISTENER_MOCK_HPP_

#include <iostream>

#include <aruwlib/communication/can/can_rx_listener.hpp>
#include <gmock/gmock.h>
#include <modm/architecture/interface/can_message.hpp>

namespace aruwlib
{
namespace mock
{
class CanRxListenerMock : public aruwlib::can::CanRxListener
{
public:
    CanRxListenerMock(aruwlib::Drivers* drivers, uint32_t id, aruwlib::can::CanBus bus)
        : aruwlib::can::CanRxListener(drivers, id, bus)
    {
    }
    MOCK_METHOD(void, processMessage, (const modm::can::Message& message), (override));

};  // class CanRxListenerMock
}  // namespace mock
}  // namespace aruwlib

#endif  //  CAN_RX_LISTENER_MOCK_HPP_
