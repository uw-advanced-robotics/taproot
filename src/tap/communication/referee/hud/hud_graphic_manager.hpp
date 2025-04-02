#pragma once

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
    HudGraphicManager(RefSerialTransmitter transmitter);

    void nextAvailableName(uint8_t name[3]);

    void needsUpdate(HudGraphic* graphic);
    void needsUpdate(Character* graphic);

    void clearAllLayers() { this->layersToDelete = 1 << 10; }
    void clearLayer(uint8_t layer) { this->layersToDelete = 1 << layer; }

    bool run();

private:
    RefSerialTransmitter transmitter;

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