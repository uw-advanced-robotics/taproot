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

#include "hud_graphic_manager.hpp"

#include "tap/drivers.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#define DRAW_LOOP(count)\
    index = 0;\
    while (index < count)\
    {\
        graphic ## count.graphicData[index++] = draw->graphic;\
        draw->changed = false;\
        draw->added = true; \
        draw = draw->nextDraw;\
    }\
    toDraw -= count;\
    PT_CALL(transmitter.sendGraphic(&graphic ## count));\

namespace tap::hud
{
HudGraphicManager::HudGraphicManager(RefSerialTransmitter& transmitter):
    transmitter(transmitter)
{}


static uint32_t currentName = 0;
void HudGraphicManager::nextAvailableName(uint8_t name[3])
{
    if (currentName > 0xffffff)
    {
        return;
    }
    else
    {
        name[0] = static_cast<uint8_t>((currentName >> 16) & 0xff);
        name[1] = static_cast<uint8_t>((currentName >> 8) & 0xff);
        name[2] = static_cast<uint8_t>(currentName & 0xff);
        currentName++;
    }
}

void HudGraphicManager::needsUpdate(HudGraphic* graphic)
{
    if (draw == nullptr)
    {
        draw = graphic;
        drawEnd = graphic;
        toDraw = 1;
    }
    else
    {
        drawEnd->nextDraw = graphic;
        drawEnd = graphic;
        toDraw += 1;
    }
}

void HudGraphicManager::needsUpdate(Character* graphic)
{
    if (drawCharacter == nullptr)
    {
        drawCharacter = graphic;
        drawEndCharacter = graphic;
    }
    else
    {
        drawEndCharacter->nextDraw = graphic;
        drawEndCharacter = graphic;
    }
}

bool HudGraphicManager::run()
{
    PT_BEGIN();
    while (true)
    {
        if (layersToDelete & (1 << 10))
        {
            std::cout<< "del all";
            PT_CALL(transmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
        }
        else
        {
            index = 0;
            while (layersToDelete != 0 && index < 10)
            {
                if (layersToDelete & (1 << index))
                {
                    std::cout << "del " << index;
                    PT_CALL(transmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_GRAPHIC_LAYER, index));
                    layersToDelete &= ~(1 << index);
                    index += 1;
                }
            }
        }

        if (drawCharacter != nullptr)
        {
            graphicCharacter.graphicData = drawCharacter->graphic;
            strncpy(graphicCharacter.msg, drawCharacter->message, 30); 
            draw->changed = false;
            draw->added = true;
            drawCharacter = drawCharacter->nextDraw;
            PT_CALL(transmitter.sendGraphic(&graphicCharacter));
        }

        while (toDraw != 0)
        {
            if (toDraw >= 7)
            {
                DRAW_LOOP(7);
            }
            else if (toDraw >= 5)
            {
                DRAW_LOOP(5);
            }
            else if (toDraw >= 2)
            {
                DRAW_LOOP(2);
            }
            else
            {
                graphic1.graphicData = draw->graphic;
                draw->changed = false;
                draw->added = true;
                draw = draw->nextDraw;
                toDraw -= 1;
                PT_CALL(transmitter.sendGraphic(&graphic1));
            }
        }
        PT_YIELD();
    }
    PT_END();
}
}