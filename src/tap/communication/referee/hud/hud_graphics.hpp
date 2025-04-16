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

#ifndef TAPROOT_HUD_GRAPHICS_HPP_
#define TAPROOT_HUD_GRAPHICS_HPP_

#include "hud_graphic.hpp"

namespace tap::hud
{
    class Line : public HudGraphic
    {
    public:
        Line(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            uint32_t endX,
            uint32_t endY,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 5,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setEndX(uint32_t x);

        void setEndY(uint32_t y);
    };

    class Rectangle : public HudGraphic
    {
    public:
        Rectangle(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 5,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setX(uint32_t x) override;

        void setY(uint32_t y) override;
        
        void setWidth(uint32_t width);

        void setHeight(uint32_t height);
    };

    class Circle : public HudGraphic
    {
    public:
        Circle(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            uint32_t radius,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 5,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setRadius(uint32_t radius);
    };

    class Ellipse : public Rectangle
    {
    public:
        Ellipse(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 5,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);
    };

    class Arc : public Ellipse
    {
    public:
        Arc(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            uint32_t startAngle,
            uint32_t endAngle,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 5,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setStartAngle(uint32_t startAngle);

        void setEndAngle(uint32_t endAngle);
    };

    class Integer : public HudGraphic
    {
    public:
        Integer(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            int32_t value,
            uint32_t fontSize = 14,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 3,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setValue(int32_t value);
        
        void setFontSize(uint32_t fontSize);
    };

    class Float : public HudGraphic
    {
    public:
        Float(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            float value,
            uint32_t fontSize = 14,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 3,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setValue(float value);

        void setFontSize(uint32_t fontSize);
    };

    class Character : public HudGraphic
    {
    friend class HudGraphicManager;
    public:
        Character(HudGraphicManager& manager,
            uint32_t x,
            uint32_t y,
            char* message,
            uint32_t length,
            uint32_t fontSize = 14,
            RefSerialData::Tx::GraphicColor color = RefSerialData::Tx::GraphicColor::WHITE,
            uint32_t lineWidth = 3,
            uint32_t layer = 0,
            uint32_t refreshMillis = 500,
            bool showOnCreation = true);

        void setMessage(char* message, uint32_t length);

        void setFontSize(uint32_t fontSize);

        void setLayer(uint32_t layer) override;
        void setColor(RefSerialData::Tx::GraphicColor color) override;
        void setLineWidth(uint32_t lineWidth) override;
        void setX(uint32_t x) override;
        void setY(uint32_t y) override;
        void hideGraphic() override;
        void showGraphic() override;

#ifndef ENV_UNIT_TESTS
    protected:
#endif
        char message[30];

        Character* nextDraw;
    };
}

#endif // TAPROOT_HUD_GRAPHICS_HPP_