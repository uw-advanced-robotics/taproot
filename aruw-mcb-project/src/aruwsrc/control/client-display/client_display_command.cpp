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

#include "client_display_command.hpp"

#include "aruwlib/Drivers.hpp"

#include "client_display_subsystem.hpp"

using namespace aruwlib::control;
using namespace aruwlib::serial;
using aruwlib::Drivers;

#define delay()                                  \
    delayTimer.restart(DELAY_PERIOD_BTWN_SENDS); \
    RF_WAIT_UNTIL(delayTimer.execute());

namespace aruwsrc::display
{
ClientDisplayCommand::ClientDisplayCommand(
    Drivers *drivers,
    ClientDisplaySubsystem *clientDisplay,
    const Command *wiggleCommand,
    const Command *followTurretCommand,
    const Command *beybladeCommand,
    const Command *baseDriveCommand)
    : Command(),
      drivers(drivers),
      wiggleCommand(wiggleCommand),
      followTurretCommand(followTurretCommand),
      beybladeCommand(beybladeCommand),
      baseDriveCommand(baseDriveCommand),
      driveCommandMsg()
{
    addSubsystemRequirement(clientDisplay);
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();
    PT_CALL(initializeNonblocking());
    while (true)
    {
        PT_CALL(updateDriveCommandMsg());
        // PT_CALL(updateCapBankMsg());
        PT_CALL(updateTurretReticleMsg());
        PT_YIELD();
    }
    PT_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::initializeNonblocking()
{
    RF_BEGIN(0);
    RF_WAIT_WHILE(drivers->refSerial.getRobotData().robotId == RefSerial::RobotId::INVALID);
    initDriveCommandMsg();
    initTurretReticleMsg();
    initCapBankMsg();
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(1);
    // Check if updating is necessary
    if (drivers->commandScheduler.isCommandScheduled(wiggleCommand))
    {
        newDriveCommandScheduled = wiggleCommand;
        driveCommandColor = RefSerial::YELLOW;
    }
    else if (drivers->commandScheduler.isCommandScheduled(followTurretCommand))
    {
        newDriveCommandScheduled = followTurretCommand;
        driveCommandColor = RefSerial::ORANGE;
    }
    else if (drivers->commandScheduler.isCommandScheduled(beybladeCommand))
    {
        newDriveCommandScheduled = beybladeCommand;
        driveCommandColor = RefSerial::PURPLISH_RED;
    }
    else if (drivers->commandScheduler.isCommandScheduled(baseDriveCommand))
    {
        newDriveCommandScheduled = baseDriveCommand;
        driveCommandColor = RefSerial::GREEN;
    }

    if (addDriveCommandTimer.execute())
    {
        // Add the graphic
        drivers->refSerial.configGraphicGenerics(
            &driveCommandMsg.graphicData,
            DRIVE_TEXT_NAME,
            RefSerial::ADD_GRAPHIC,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            driveCommandColor);

        drivers->refSerial.configCharacterMsg(
            FONT_SIZE,
            400,
            LINE_THICKNESS,
            SCREEN_MARGIN,
            TEXT_TOP_ROW_Y,
            "",
            &driveCommandMsg);

        drivers->refSerial.sendGraphic(&driveCommandMsg);
        delay();
    }
    else if ((newDriveCommandScheduled != currDriveCommandScheduled &&
              newDriveCommandScheduled != nullptr))
    {
        currDriveCommandScheduled = newDriveCommandScheduled;

        // Modify the graphic
        drivers->refSerial.configGraphicGenerics(
            &driveCommandMsg.graphicData,
            DRIVE_TEXT_NAME,
            RefSerial::ADD_GRAPHIC_MODIFY,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            driveCommandColor);

        drivers->refSerial.configCharacterMsg(
            FONT_SIZE,
            400,
            LINE_THICKNESS,
            SCREEN_MARGIN,
            TEXT_TOP_ROW_Y,
            currDriveCommandScheduled->getName(),
            &driveCommandMsg);

        drivers->refSerial.sendGraphic(&driveCommandMsg);
        delay();
    }

    // No delay necessary since didn't send anything
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateCapBankMsg()
{
    RF_BEGIN(2);

    if (sendCapBankTimer.execute())
    {
        drivers->refSerial.sendGraphic(&capStringMsg, false, true);

        delay();

        capPowerRemainMsg.graphicData.operation = RefSerial::ADD_GRAPHIC;
        drivers->refSerial.updateInteger(capicatance++, &capPowerRemainMsg.graphicData);
        drivers->refSerial.sendGraphic(&capPowerRemainMsg);
        capPowerRemainMsg.graphicData.operation = RefSerial::ADD_GRAPHIC_MODIFY;
    }
    else
    {
        drivers->refSerial.updateInteger(capicatance++, &capPowerRemainMsg.graphicData);
        drivers->refSerial.sendGraphic(&capPowerRemainMsg);
    }

    delay();
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateTurretReticleMsg()
{
    RF_BEGIN(3);
    delay();
    if (sendReticleTimer.execute())
    {
        drivers->refSerial.sendGraphic(&reticleMsg, false, true);
        delay();
    }
    RF_END();
}

void ClientDisplayCommand::initCapBankMsg()
{
    drivers->refSerial.configGraphicGenerics(
        &capStringMsg.graphicData,
        CAP_TEXT_NAME,
        RefSerial::ADD_GRAPHIC,
        CAP_BANK_LAYER_1,
        RefSerial::YELLOW);

    drivers->refSerial.configCharacterMsg(
        FONT_SIZE,
        200,
        FONT_THICKNESS,
        SCREEN_WIDTH - 400,
        TEXT_TOP_ROW_Y,
        "Capacitance:",
        &capStringMsg);
    drivers->refSerial.sendGraphic(&capStringMsg, true, false);

    drivers->refSerial.configGraphicGenerics(
        &capPowerRemainMsg.graphicData,
        CAP_VALUE_NAME,
        RefSerial::ADD_GRAPHIC_MODIFY,
        CAP_BANK_LAYER_2,
        RefSerial::YELLOW);

    drivers->refSerial.configInteger(
        FONT_SIZE + 10,  // Slightly larger than other text
        FONT_THICKNESS,
        SCREEN_WIDTH - 350,
        TEXT_TOP_ROW_Y - 60,
        capicatance++,
        &capPowerRemainMsg.graphicData);
}

void ClientDisplayCommand::initDriveCommandMsg()
{
    drivers->refSerial.configGraphicGenerics(
        &driveCommandMsg.graphicData,
        DRIVE_TEXT_NAME,
        RefSerial::ADD_GRAPHIC,
        DRIVE_COMMAND_GRAPHIC_LAYER,
        driveCommandColor);

    drivers->refSerial.configCharacterMsg(
        FONT_SIZE,
        400,
        LINE_THICKNESS,
        SCREEN_MARGIN,
        TEXT_TOP_ROW_Y,
        "",
        &driveCommandMsg);
}

void ClientDisplayCommand::initTurretReticleMsg()
{
    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[0],
        RETICLE_LINE1_NAME,
        RefSerial::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[1],
        RETICLE_LINE2_NAME,
        RefSerial::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[2],
        RETICLE_LINE3_NAME,
        RefSerial::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[3],
        RETICLE_LINE4_NAME,
        RefSerial::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[4],
        RETICLE_CIRCLE_NAME,
        RefSerial::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::YELLOW);

    drivers->refSerial.configLine(
        LINE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_1M_WIDTH / 2,
        TURRET_RETICLE_1MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_1M_WIDTH / 2,
        TURRET_RETICLE_1MY,
        &reticleMsg.graphicData[0]);

    drivers->refSerial.configLine(
        LINE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_3M_WIDTH / 2,
        TURRET_RETICLE_3MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_3M_WIDTH / 2,
        TURRET_RETICLE_3MY,
        &reticleMsg.graphicData[1]);

    drivers->refSerial.configLine(
        LINE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_5M_WIDTH / 2,
        TURRET_RETICLE_5MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_5M_WIDTH / 2,
        TURRET_RETICLE_5MY,
        &reticleMsg.graphicData[2]);

    drivers->refSerial.configLine(
        LINE_THICKNESS,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_1MY,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_5MY - 50,
        &reticleMsg.graphicData[3]);

    drivers->refSerial.configCircle(
        LINE_THICKNESS,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_1MY,
        5,
        &reticleMsg.graphicData[4]);

    // Computes crcs which won't be changing in the future
    drivers->refSerial.sendGraphic(&reticleMsg, true, false);
}
}  // namespace aruwsrc::display
