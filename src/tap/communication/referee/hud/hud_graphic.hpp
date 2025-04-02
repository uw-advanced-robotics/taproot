#pragma once
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

protected:
    bool changed = false, queued = false;

    bool updateGraphicOp();

    RefSerialData::Tx::GraphicData graphic{};
    HudGraphic* nextDraw;
    HudGraphicManager& manager;

    tap::arch::PeriodicMilliTimer refreshTimer;
};
}