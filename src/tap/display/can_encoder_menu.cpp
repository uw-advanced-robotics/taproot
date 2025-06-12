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

#include "can_encoder_menu.hpp"

#include <algorithm>
#include <cmath>

#include "tap/drivers.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"

using namespace tap::encoder;
using namespace tap::can;

namespace tap
{
namespace display
{

CanEncoder* CAN_ENCODERS[2][8] = {{}, {}};

void displayCanEncoder(CanBus bus, CanEncoder* encoder)
{
    CAN_ENCODERS[static_cast<uint8_t>(bus)][encoder->canIdentifier - CanEncoderId::ID0] = encoder;
}

CanEncoderMenu::CanEncoderMenu(
    modm::ViewStack<DummyAllocator<modm::IAbstractView> >* stack,
    Drivers* drivers,
    int entriesToDisplay)
    : modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >(stack, CAN_ENCODER_MENU_ID),
      drivers(drivers),
      verticalScroll(drivers, (CanEncoderId::ID7 - CanEncoderId::ID0 + 1) * 2, entriesToDisplay)
{
}

void CanEncoderMenu::drawEncoder(CanBus canBus, uint8_t rawId)
{
    const CanEncoder* encoder = nullptr;
    const char* canBusName = "CAN?";
    bool printCursor = false;
    switch (canBus)
    {
        case CanBus::CAN_BUS1:
            encoder = CAN_ENCODERS[static_cast<uint8_t>(canBus)][rawId];
            canBusName = "CAN1";
            printCursor = (rawId == verticalScroll.getCursorIndex());
            break;
        case CanBus::CAN_BUS2:
            encoder = CAN_ENCODERS[static_cast<uint8_t>(canBus)][rawId];
            canBusName = "CAN2";
            printCursor =
                ((rawId + 8) ==
                 verticalScroll.getCursorIndex());
            break;
    }

    getViewStack()->getDisplay() << (printCursor ? ">" : " ") << canBusName << " Encoder "
                                 << (rawId);

    if (encoder != nullptr)
    {
        if (encoder->isOnline())
        {
            getViewStack()->getDisplay() << ": ON ";

            getViewStack()->getDisplay() << encoder->getEncoder().getWrappedValue();
        }
        else
        {
            getViewStack()->getDisplay() << ": OFF";
        }
    }
    else
    {
        getViewStack()->getDisplay() << ": NULL";
    }
    getViewStack()->getDisplay() << modm::endl;
}

void CanEncoderMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    auto can1MinIndex = verticalScroll.getSmallestIndexDisplayed();
    auto can1MaxIndex = std::min(
        CanEncoderId::ID7 - CanEncoderId::ID0,
        static_cast<int>(verticalScroll.getLargestIndexDisplayed()));
    for (uint8_t id = can1MinIndex; id <= can1MaxIndex; id++)
    {
        drawEncoder(CanBus::CAN_BUS1, id);
    }

    auto can2MinIndex = std::max(
        0,
        verticalScroll.getSmallestIndexDisplayed() - CanEncoderId::ID7 - CanEncoderId::ID0 + 1);
    auto can2MaxIndex = std::min(
        CanEncoderId::ID7 - CanEncoderId::ID0,
        verticalScroll.getLargestIndexDisplayed() - CanEncoderId::ID7 - CanEncoderId::ID0 + 1);
    for (uint8_t id = can2MinIndex; id <= can2MaxIndex; id++)
    {
        drawEncoder(CanBus::CAN_BUS2, id);
    }
}

void CanEncoderMenu::update() {}

bool CanEncoderMenu::hasChanged()
{
    uint8_t newCan1Status = 0;
    uint8_t newCan2Status = 0;
    for (uint8_t id = 0; id <= CanEncoderId::ID7 - CanEncoderId::ID0; id++)
    {
        CanEncoder const* encoder = CAN_ENCODERS[static_cast<uint8_t>(CanBus::CAN_BUS1)][id];
        if (encoder != nullptr && encoder->isOnline())
        {
            newCan1Status |= (1 << (id));
        }

        encoder = CAN_ENCODERS[static_cast<uint8_t>(CanBus::CAN_BUS1)][id];
        if (encoder != nullptr && encoder->isOnline())
        {
            newCan2Status |= (1 << (id));
        }
    }

    bool motorStatusChanged = (can1PrevDisplayedStatus != newCan1Status) ||
                              (can2PrevDisplayedStatus != newCan2Status);

    can1PrevDisplayedStatus = newCan1Status;
    can2PrevDisplayedStatus = newCan2Status;

    return verticalScroll.acknowledgeCursorChanged() || motorStatusChanged;
}

void CanEncoderMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            verticalScroll.onShortButtonPress(modm::MenuButtons::DOWN);
            break;
        case modm::MenuButtons::UP:
            verticalScroll.onShortButtonPress(modm::MenuButtons::UP);
            break;
        case modm::MenuButtons::OK:
            break;
    }
}
}  // namespace display
}  // namespace tap
