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

#ifndef TAPROOT_HUD_GRAPHIC_HPP_
#define TAPROOT_HUD_GRAPHIC_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/architecture/periodic_timer.hpp"


namespace tap::hud
{

class HudGraphicManager;

using namespace tap::communication::serial;

class HudGraphic
{
friend class HudGraphicManager;
public:
    HudGraphic(HudGraphicManager& manager, 
        RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE, 
        uint32_t lineWidth = 5,
        uint32_t layer = 0,
        uint32_t refreshMillis = 500,
        bool showOnCreation = true);

    virtual void setLayer(uint32_t layer);

    virtual void setColor(RefSerialData::Tx::GraphicColor color);

    virtual void setLineWidth(uint32_t lineWidth);

    virtual void setX(uint32_t x);

    virtual void setY(uint32_t y);

    virtual void hideGraphic();

    virtual void showGraphic();

    void setRefreshRate(uint32_t millis) { refreshTimer.restart(millis);}

    bool needsRedraw() { return changed && refreshTimer.execute(); }

    RefSerialData::Tx::GraphicData* getGraphic() { return &this->graphic; }

#ifndef ENV_UNIT_TESTS
    protected:
#endif
    bool changed = false, added = false;

    bool updateGraphicOp();

    RefSerialData::Tx::GraphicData graphic{};
    HudGraphic* nextDraw;
    HudGraphicManager& manager;

    tap::arch::PeriodicMilliTimer refreshTimer;
};
}

#endif // TAPROOT_HUD_GRAPHIC_HPP_
