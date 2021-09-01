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

#include "mpu6500_terminal_serial_handler.hpp"

#include "tap/algorithms/strtok.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace sensors
{
constexpr char Mpu6500TerminalSerialHandler::USAGE[];

void Mpu6500TerminalSerialHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

bool Mpu6500TerminalSerialHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    char* arg;
    printingAngles = false;
    printingGyro = false;
    printingAccel = false;
    printingTemp = false;
    while (
        (arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine)))
    {
        if (!printingAngles && strcmp(arg, "angle") == 0)
        {
            printingAngles = true;
        }
        else if (!printingGyro && strcmp(arg, "gyro") == 0)
        {
            printingGyro = true;
        }
        else if (!printingAccel && strcmp(arg, "accel") == 0)
        {
            printingAccel = true;
        }
        else if (!printingTemp && strcmp(arg, "temp") == 0)
        {
            printingTemp = true;
        }
        else if (strcmp(arg, "-h"))
        {
            outputStream << USAGE;
            return false;
        }
    }

    if (!printingAngles && !printingGyro && !printingAccel && !printingTemp)
    {
        outputStream << USAGE;
        return !streamingEnabled;
    }

    printHeader(outputStream);

    terminalSerialStreamCallback(outputStream);
    return true;
}

static inline void checkNeedsTab(bool& needsTab, modm::IOStream& outputStream)
{
    if (needsTab)
    {
        outputStream << "\t";
    }
    else
    {
        needsTab = true;
    }
}

void Mpu6500TerminalSerialHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    bool needsTab = false;
    Mpu6500& mpu = drivers->mpu6500;
    if (printingAngles)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getPitch()),
            static_cast<double>(mpu.getRoll()),
            static_cast<double>(mpu.getYaw()));
    }
    if (printingGyro)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getGx()),
            static_cast<double>(mpu.getGy()),
            static_cast<double>(mpu.getGz()));
    }
    if (printingAccel)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getAx()),
            static_cast<double>(mpu.getAy()),
            static_cast<double>(mpu.getAz()));
    }
    if (printingTemp)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf("%.2f", static_cast<double>(mpu.getTemp()));
    }
    outputStream << modm::endl;
}

void Mpu6500TerminalSerialHandler::printHeader(modm::IOStream& outputStream)
{
    bool needsTab = false;
    if (printingAngles)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "pit\trol\tyaw";
    }
    if (printingGyro)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "gx\tgy\tgz";
    }
    if (printingAccel)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "ax\tay\taz";
    }
    if (printingTemp)
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "temp";
    }
    outputStream << modm::endl;
}
}  // namespace sensors
}  // namespace tap
