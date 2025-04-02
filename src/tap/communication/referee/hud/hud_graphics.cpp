#include "hud_graphic.hpp"
#include "hud_graphics.hpp"
#include "hud_graphic_manager.hpp"

#define UPDATE_GRAPHIC_FIELD_BASE(field, val, method) \
        this->changed |= static_cast<uint32_t>(this->graphic.field) != static_cast<uint32_t>(val);\
        this->updateGraphicOp();\
        this->graphic.field = val;\
        if (this->needsRedraw() && !this->queued)\
        {\
            this->manager.method(this);\
        }\

#define UPDATE_GRAPHIC_FIELD(field, val)\
    UPDATE_GRAPHIC_FIELD_BASE(field, val, needsUpdate)

#define UPDATE_CHARACTER_GRAPHIC_FIELD(field, val) \
    UPDATE_GRAPHIC_FIELD_BASE(field, val, needsUpdate)

namespace tap::hud
{
HudGraphic::HudGraphic(HudGraphicManager& manager,
        RefSerialData::Tx::GraphicColor color, 
        uint32_t lineWidth,
        uint32_t layer,
        uint32_t refreshMillis,
        bool showOnCreation):
    nextDraw(nullptr),
    manager(manager),
    refreshTimer(refreshMillis)
{
    manager.nextAvailableName(this->graphic.name);

    if (showOnCreation)
    {
        this->showGraphic();
    }

    this->setColor(color);
    this->setLineWidth(lineWidth);
    this->setLayer(layer);
}

void HudGraphic::setLayer(uint32_t layer)
{
    UPDATE_GRAPHIC_FIELD(layer, layer);
}

void HudGraphic::setColor(RefSerialData::Tx::GraphicColor color)
{
    UPDATE_GRAPHIC_FIELD(color, static_cast<uint8_t>(color));
}

void HudGraphic::setLineWidth(uint32_t lineWidth)
{
    UPDATE_GRAPHIC_FIELD(lineWidth, lineWidth);
}

void HudGraphic::setX(uint32_t x)
{
    UPDATE_GRAPHIC_FIELD(startX, x);
}

void HudGraphic::setY(uint32_t y)
{
    UPDATE_GRAPHIC_FIELD(startY, y);
}

void HudGraphic::hideGraphic()
{
    if (this->graphic.operation != RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE)
    {
        this->graphic.operation = RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE;
        this->manager.needsUpdate(this);
    } 
}

void HudGraphic::showGraphic()
{
    if (this->graphic.operation == RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE)
    {
        this->graphic.operation = RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD;
        this->manager.needsUpdate(this);
    } 
}

bool HudGraphic::updateGraphicOp()
{
    if (this->graphic.operation == RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD)
    {
        this->graphic.operation = RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY;
        return true;
    } 
    return false;
}

// Line
Line::Line(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            uint32_t endX,
            uint32_t endY,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::STRAIGHT_LINE);

    this->setX(x);
    this->setY(y);
    this->setEndX(endX);
    this->setEndY(endY);
}

void Line::setEndX(uint32_t x)
{
    UPDATE_GRAPHIC_FIELD(endX, x);
}

void Line::setEndY(uint32_t y)
{
    UPDATE_GRAPHIC_FIELD(endY, y);
}

// Rectangle
Rectangle::Rectangle(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::RECTANGLE);

    this->setX(x);
    this->setY(y);
    this->setWidth(width);
    this->setHeight(height);
}

void Rectangle::setWidth(uint32_t width)
{
    UPDATE_GRAPHIC_FIELD(endX, this->graphic.startX + width);
}

void Rectangle::setHeight(uint32_t height)
{
    UPDATE_GRAPHIC_FIELD(endY, this->graphic.startY + height);
}

void Rectangle::setX(uint32_t x)
{
    int32_t width = this->graphic.endX - this->graphic.startX;
    HudGraphic::setX(x);
    this->graphic.endX = this->graphic.startX + width;
}

void Rectangle::setY(uint32_t y)
{
    int32_t height = this->graphic.endY - this->graphic.startY;
    HudGraphic::setY(y);
    this->graphic.endY = this->graphic.startY + height;
}

// Circle
Circle::Circle(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            uint32_t radius,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::CIRCLE);

    this->setX(x);
    this->setY(y);
    this->setRadius(radius);
}

void Circle::setRadius(uint32_t radius)
{
    UPDATE_GRAPHIC_FIELD(radius, radius);
}

// Ellipse
Ellipse::Ellipse(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : Rectangle(manager, x, y, width, height, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::ELLIPSE);
}

// Arc
Arc::Arc(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            uint32_t width,
            uint32_t height,
            uint32_t startAngle,
            uint32_t endAngle,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : Ellipse(manager, x, y, width, height, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::ARC);

    this->setStartAngle(startAngle);
    this->setEndAngle(endAngle);
}

void Arc::setStartAngle(uint32_t startAngle)
{
    UPDATE_GRAPHIC_FIELD(startAngle, startAngle);
}

void Arc::setEndAngle(uint32_t endAngle)
{
    UPDATE_GRAPHIC_FIELD(endAngle, endAngle);
}

// Integer
Integer::Integer(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            int32_t value,
            uint32_t fontSize,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::INTEGER);

    this->setX(x);
    this->setY(y);
    this->setValue(value);
    this->setFontSize(fontSize);
}

void Integer::setValue(int32_t value)
{
    UPDATE_GRAPHIC_FIELD(value, value);
}

void Integer::setFontSize(uint32_t fontSize)
{
    UPDATE_GRAPHIC_FIELD(startAngle, fontSize);
}

// Float
Float::Float(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            float value,
            uint32_t fontSize,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::FLOATING_NUM);

    this->setX(x);
    this->setY(y);
    this->setValue(value);
    this->setFontSize(fontSize);
}

void Float::setValue(float value)
{
    UPDATE_GRAPHIC_FIELD(value, static_cast<int32_t>(value * 1000));
}

void Float::setFontSize(uint32_t fontSize)
{
    UPDATE_GRAPHIC_FIELD(startAngle, fontSize);
}

// Character
Character::Character(HudGraphicManager& manager, 
            uint32_t x,
            uint32_t y,
            char* message,
            uint32_t length,
            uint32_t fontSize,
            RefSerialData::Tx::GraphicColor color,
            uint32_t lineWidth,
            uint32_t layer,
            uint32_t refreshMillis,
            bool showOnCreation) : HudGraphic(manager, color, lineWidth, layer, refreshMillis, showOnCreation),
                nextDraw(nullptr)
{
    graphic.type = static_cast<uint8_t>(RefSerial::Tx::GraphicType::CHARACTER);

    this->setX(x);
    this->setY(y);
    this->setMessage(message, length);
    this->setFontSize(fontSize);
}

void Character::setMessage(char* message, uint32_t length)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(endAngle, length);
    this->changed |= strncmp(this->message, message, length);
    strncpy(this->message, message, length);
    if (this->needsRedraw() && !this->queued)
    {
        this->manager.needsUpdate(this);
    }
}

void Character::setFontSize(uint32_t fontSize)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(startAngle, fontSize);
}

void Character::setLayer(uint32_t layer)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(layer, layer);
}

void Character::setColor(RefSerialData::Tx::GraphicColor color)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(color, static_cast<uint8_t>(color));
}

void Character::setLineWidth(uint32_t lineWidth)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(lineWidth, lineWidth);
}

void Character::setX(uint32_t x)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(startX, x);
}

void Character::setY(uint32_t y)
{
    UPDATE_CHARACTER_GRAPHIC_FIELD(startY, y);
}

void Character::hideGraphic()
{
    if (this->graphic.operation != RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE)
    {
        this->graphic.operation = RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE;
        this->manager.needsUpdate(this);
    } 
}

void Character::showGraphic()
{
    if (this->graphic.operation == RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE)
    {
        this->graphic.operation = RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD;
        this->manager.needsUpdate(this);
    } 
}
}