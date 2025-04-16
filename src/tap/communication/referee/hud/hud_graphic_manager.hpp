/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_HUD_GRAPHIC_MANAGER_HPP_
#define TAPROOT_HUD_GRAPHIC_MANAGER_HPP_

#include "hud_graphic.hpp"
#include "hud_graphics.hpp"
#include "modm/processing/protothread.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

namespace tap::hud
{

using namespace tap::communication::serial;
class HudGraphicManager : modm::pt::Protothread
{
public:
    HudGraphicManager(RefSerialTransmitter& transmitter);

    void nextAvailableName(uint8_t name[3]);

    void needsUpdate(HudGraphic* graphic);
    void needsUpdate(Character* graphic);

    void clearAllLayers() { this->layersToDelete = 1 << 10; }
    void clearLayer(uint8_t layer) { this->layersToDelete = 1 << layer; }

    bool run();

private:
    RefSerialTransmitter& transmitter;

    HudGraphic* draw = nullptr;
    HudGraphic* drawEnd = nullptr;
    uint8_t toDraw = 0;

    Character* drawCharacter = nullptr;
    Character* drawEndCharacter = nullptr;

    RefSerialData::Tx::Graphic1Message graphic1{};
    RefSerialData::Tx::Graphic2Message graphic2{};
    RefSerialData::Tx::Graphic5Message graphic5{};
    RefSerialData::Tx::Graphic7Message graphic7{};
    RefSerialData::Tx::GraphicCharacterMessage graphicCharacter{};

    uint16_t layersToDelete = 0;

    int index = 0;
};
}

#endif // TAPROOT_HUD_GRAPHIC_MANAGER_HPP_
