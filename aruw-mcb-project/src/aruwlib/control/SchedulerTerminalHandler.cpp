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

#include "SchedulerTerminalHandler.hpp"

#include <algorithm>

#include "aruwlib/Drivers.hpp"
#include "aruwlib/algorithms/strtok.hpp"

namespace aruwlib
{
namespace control
{
constexpr char SchedulerTerminalHandler::HEADER[];
constexpr char SchedulerTerminalHandler::USAGE[];

SchedulerTerminalHandler::SchedulerTerminalHandler(Drivers* driver) : drivers(driver) {}

void SchedulerTerminalHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

void SchedulerTerminalHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    printInfo(outputStream);
}

bool SchedulerTerminalHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool)
{
    char* arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);

    if (arg == nullptr || strcmp(arg, "-H") == 0)
    {
        outputStream << USAGE;
        return arg != nullptr;
    }
    else if (strcmp(arg, "allsubcmd") == 0)
    {
        printInfo(outputStream);
        return true;
    }
    else
    {
        outputStream << "Command not found, try again, type \"scheduler -H\" for more."
                     << modm::endl;
        return false;
    }
}

void SchedulerTerminalHandler::printInfo(modm::IOStream& outputStream)
{
    outputStream << "Subsystems:" << modm::endl;
    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](Subsystem* sub) { outputStream << " " << sub->getName() << modm::endl; });

    outputStream << "Commands:" << modm::endl;
    std::for_each(
        drivers->commandScheduler.cmdMapBegin(),
        drivers->commandScheduler.cmdMapEnd(),
        [&](Command* cmd) { outputStream << " " << cmd->getName() << modm::endl; });
}
}  // namespace control

}  // namespace aruwlib
