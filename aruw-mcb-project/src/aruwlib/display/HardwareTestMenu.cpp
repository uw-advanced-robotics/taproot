/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "HardwareTestMenu.hpp"

#include "aruwlib/control/command_scheduler.hpp"
#include "aruwlib/control/subsystem.hpp"

namespace aruwlib
{
namespace display
{
HardwareTestMenu::HardwareTestMenu(modm::ViewStack* vs, Drivers* drivers)
    : AbstractMenu(vs, 2),
      drivers(drivers)
{
    drivers->commandScheduler.runSubsystemTests();
}

void HardwareTestMenu::update()
{
    if (this->updateHasChanged())
    {
        this->draw();
    }
}

void HardwareTestMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->getViewStack()->pop();
            drivers->commandScheduler.stopHardwareTests();
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            selectedSubsystem++;
            if (selectedSubsystem == topIndex)
            {
                bottomIndex++;
                topIndex++;
            }
            break;
        case modm::MenuButtons::UP:
            selectedSubsystem--;
            if (selectedSubsystem == bottomIndex)
            {
                bottomIndex--;
                topIndex--;
            }
            break;
        case modm::MenuButtons::OK:
            int subsystemIndex = 0;
            for (auto& subsystemToCommand : drivers->commandScheduler.getSubsystemToCommandMap())
            {
                if (subsystemIndex++ == selectedSubsystem)
                {
                    subsystemToCommand.first->setHardwareTestsComplete();
                    break;
                }
            }
            break;
    }

    if (selectedSubsystem < 0)
    {
        selectedSubsystem = 0;
    }
    else if (
        selectedSubsystem >=
        static_cast<int>(drivers->commandScheduler.getSubsystemToCommandMap().size()))
    {
        selectedSubsystem = drivers->commandScheduler.getSubsystemToCommandMap().size() - 1;
    }

    changed = true;
}

bool HardwareTestMenu::hasChanged()
{
    bool temp = changed;
    changed = false;
    return temp;
}

bool HardwareTestMenu::updateHasChanged()
{
    int i = 0;
    uint64_t changedSubsystems = 0;
    for (auto& subsystemToCommand : drivers->commandScheduler.getSubsystemToCommandMap())
    {
        changedSubsystems += (subsystemToCommand.first->isHardwareTestComplete() ? 1 : 0) << i;
    }

    changed = changed || changedSubsystems != completeSubsystems;
    completeSubsystems = changedSubsystems;
    return changed;
}

void HardwareTestMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << HardwareTestMenu::getMenuName() << modm::endl;

    int subsystemIndex = 0;
    for (auto& subsystemToCommand : drivers->commandScheduler.getSubsystemToCommandMap())
    {
        aruwlib::control::Subsystem* subsystem = subsystemToCommand.first;
        if (subsystemIndex <= topIndex && subsystemIndex >= bottomIndex)
            display << ((subsystemIndex == selectedSubsystem) ? ">" : " ")
                    << (subsystem->isHardwareTestComplete() ? "[done] " : "[not]  ")
                    << subsystem->getName() << modm::endl;
        subsystemIndex++;
    }
}
}  // namespace display
}  // namespace aruwlib
