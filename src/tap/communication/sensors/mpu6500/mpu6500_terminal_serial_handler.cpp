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

namespace tap::sensors
{
constexpr char Mpu6500TerminalSerialHandler::USAGE[];

#define SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, subject) \
    ((inspectSubjectBitmap & subject) != 0)
#define SET_INSPECT_SUBJECT(inspectSubjectBitmap, subject) (inspectSubjectBitmap |= subject)

void Mpu6500TerminalSerialHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

bool Mpu6500TerminalSerialHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    char* arg;
    inspectSubjectBitmap = 0;
    while (
        (arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine)))
    {
        if (!SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ANGLES) &&
            strcmp(arg, "angle") == 0)
        {
            SET_INSPECT_SUBJECT(inspectSubjectBitmap, InspectSubject::ANGLES);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::GYRO) &&
            strcmp(arg, "gyro") == 0)
        {
            SET_INSPECT_SUBJECT(inspectSubjectBitmap, InspectSubject::GYRO);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ACCEL) &&
            strcmp(arg, "accel") == 0)
        {
            SET_INSPECT_SUBJECT(inspectSubjectBitmap, InspectSubject::ACCEL);
        }
        else if (
            !SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::TEMP) &&
            strcmp(arg, "temp") == 0)
        {
            SET_INSPECT_SUBJECT(inspectSubjectBitmap, InspectSubject::TEMP);
        }
        else if (strcmp(arg, "-h"))
        {
            outputStream << USAGE;
            return false;
        }
    }

    if (inspectSubjectBitmap == 0)
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
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ANGLES))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getPitch()),
            static_cast<double>(mpu.getRoll()),
            static_cast<double>(mpu.getYaw()));
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::GYRO))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getGx()),
            static_cast<double>(mpu.getGy()),
            static_cast<double>(mpu.getGz()));
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ACCEL))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf(
            "%.2f\t%.2f\t%.2f",
            static_cast<double>(mpu.getAx()),
            static_cast<double>(mpu.getAy()),
            static_cast<double>(mpu.getAz()));
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::TEMP))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream.printf("%.2f", static_cast<double>(mpu.getTemp()));
    }
    outputStream << modm::endl;
}

void Mpu6500TerminalSerialHandler::printHeader(modm::IOStream& outputStream)
{
    bool needsTab = false;
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ANGLES))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "pit\trol\tyaw";
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::GYRO))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "gx\tgy\tgz";
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::ACCEL))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "ax\tay\taz";
    }
    if (SUBJECT_BEING_INSPECTED(inspectSubjectBitmap, InspectSubject::TEMP))
    {
        checkNeedsTab(needsTab, outputStream);
        outputStream << "temp";
    }
    outputStream << modm::endl;
}
}  // namespace tap::sensors
