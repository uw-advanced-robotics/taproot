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

#include <gtest/gtest.h>

#include "tap/communication/referee/hud/hud_graphics.hpp"
#include "tap/communication/referee/hud/hud_graphic_manager.hpp"
#include "tap/mock/ref_serial_transmitter_mock.hpp"
#include "tap/architecture/clock.hpp"

using namespace tap::hud;
using namespace tap::mock;
using namespace tap;
using namespace tap::communication::serial;
using namespace testing;

using namespace tap::arch::clock;

#define CREATE_GRAPHIC(graphic) \
    RefSerialData::Tx::GraphicData data; \
    memset(&data, 0, sizeof(data)); \
    RefSerialTransmitter::configGraphicGenerics( \
        &data, \
        graphic->name, \
        RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD, \
        0, \
        RefSerialData::Tx::GraphicColor::WHITE \
    );

#define COMPARE_GRAPHIC(graphic) \
    EXPECT_EQ(graphic->name[0], data.name[0]); \
    EXPECT_EQ(graphic->name[1], data.name[1]); \
    EXPECT_EQ(graphic->name[2], data.name[2]); \
    EXPECT_EQ(graphic->operation, data.operation); \
    EXPECT_EQ(graphic->type, data.type); \
    EXPECT_EQ(graphic->layer, data.layer); \
    EXPECT_EQ(graphic->color, data.color); \
    EXPECT_EQ(graphic->startAngle, data.startAngle); \
    EXPECT_EQ(graphic->endAngle, data.endAngle); \
    EXPECT_EQ(graphic->lineWidth, data.lineWidth); \
    EXPECT_EQ(graphic->startX, data.startX); \
    EXPECT_EQ(graphic->startY, data.startY); \
    EXPECT_EQ(graphic->radius, data.radius); \
    EXPECT_EQ(graphic->endX, data.endX); \
    EXPECT_EQ(graphic->endY, data.endY); 

TEST(HudGraphics, line_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Line line(manager, 0, 0, 100, 100);

    CREATE_GRAPHIC(line.getGraphic());
    RefSerialTransmitter::configLine(
        5,
        0,
        0,
        100,
        100,
        &data
    );

    COMPARE_GRAPHIC(line.getGraphic());
}

TEST(HudGraphics, rect_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Rectangle rect(manager, 0, 0, 100, 100);

    CREATE_GRAPHIC(rect.getGraphic());
    RefSerialTransmitter::configRectangle(
        5,
        0,
        0,
        100,
        100,
        &data
    );

    COMPARE_GRAPHIC(rect.getGraphic());
}

TEST(HudGraphics, circle_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Circle circle(manager, 0, 100, 100);

    CREATE_GRAPHIC(circle.getGraphic());
    RefSerialTransmitter::configCircle(
        5,
        0,
        100,
        100,
        &data
    );

    COMPARE_GRAPHIC(circle.getGraphic());
}

TEST(HudGraphics, ellipse_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Ellipse ellipse(manager, 0, 0, 100, 100);

    CREATE_GRAPHIC(ellipse.getGraphic());
    RefSerialTransmitter::configEllipse(
        5,
        0,
        0,
        100,
        100,
        &data
    );

    COMPARE_GRAPHIC(ellipse.getGraphic());
}

TEST(HudGraphics, arc_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Arc arc(manager, 0, 0, 100, 100, 10, 100);

    CREATE_GRAPHIC(arc.getGraphic());
    RefSerialTransmitter::configArc(
        10, 
        100,
        5,
        0,
        0,
        100,
        100,
        &data
    );

    COMPARE_GRAPHIC(arc.getGraphic());
}

TEST(HudGraphics, int_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Integer integer(manager, 0, 0, 100);

    CREATE_GRAPHIC(integer.getGraphic());
    RefSerialTransmitter::configInteger(
        14,
        3,
        0,
        0,
        100,
        &data
    );

    COMPARE_GRAPHIC(integer.getGraphic());
}

TEST(HudGraphics, float_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    Float float_(manager, 0, 0, 100);

    CREATE_GRAPHIC(float_.getGraphic());
    RefSerialTransmitter::configFloatingNumber(
        14,
        3,
        0,
        0,
        100.0f,
        &data
    );

    COMPARE_GRAPHIC(float_.getGraphic());
}

TEST(HudGraphics, char_message_is_correct)
{
    Drivers drivers;
    RefSerialTransmitterMock transmitter(&drivers);
    HudGraphicManager manager(transmitter);
    char message[] = "Hello, World!";
    Character character(manager, 0, 0, message, sizeof(message));

    RefSerialData::Tx::GraphicCharacterMessage charMessage;
    memset(&charMessage.graphicData, 0, sizeof(charMessage.graphicData));
    RefSerialTransmitter::configGraphicGenerics(
        &charMessage.graphicData,
        character.getGraphic()->name,
        RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD,
        0,
        RefSerialData::Tx::GraphicColor::WHITE
    );
    RefSerialTransmitter::configCharacterMsg(
        14,
        3,
        0,
        0,
        message,
        &charMessage
    );

    RefSerialData::Tx::GraphicData data = charMessage.graphicData;
    COMPARE_GRAPHIC(character.getGraphic());

    EXPECT_STREQ(character.message, charMessage.msg);
}


TEST(HudGraphics, state_logic_is_correct)
{
    ClockStub clock;
    Drivers drivers;
    NiceMock<RefSerialTransmitterMock> transmitter(&drivers);
    // transmitter.gmock3_sendGraphicImpl_44/

    EXPECT_CALL(transmitter, sendGraphic(An<RefSerialData::Tx::Graphic1Message*>(), _, _))
        .Times(3)
        .WillRepeatedly(Return(modm::ResumableResult<void>(modm::rf::Stop)));

    HudGraphicManager manager(transmitter);
    Line line(manager, 0, 0, 100, 100);

    EXPECT_TRUE(line.changed);
    EXPECT_FALSE(line.added);
    EXPECT_EQ(line.getGraphic()->operation, RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD);

    manager.run();
    clock.time = 1000;

    EXPECT_FALSE(line.changed);
    EXPECT_TRUE(line.added);

    line.setX(10);

    EXPECT_TRUE(line.changed);
    EXPECT_TRUE(line.added);
    EXPECT_EQ(line.getGraphic()->operation, RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY);

    manager.run();
    clock.time = 2000;

    EXPECT_FALSE(line.changed);
    EXPECT_TRUE(line.added);

    line.hideGraphic();

    EXPECT_TRUE(line.changed);
    EXPECT_TRUE(line.added);
    EXPECT_EQ(line.getGraphic()->operation, RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE);

    manager.run();
    clock.time = 3000;
}