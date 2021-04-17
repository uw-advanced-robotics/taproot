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

#include "HardwareTestMenu.hpp"

#include <algorithm>

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
    drivers->commandScheduler.startHardwareTests();
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
            drivers->commandScheduler.stopHardwareTests();
            this->getViewStack()->pop();
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
            for (auto it = drivers->commandScheduler.subMapBegin();
                 it != drivers->commandScheduler.subMapEnd();
                 it++)
            {
                if (subsystemIndex++ == selectedSubsystem)
                {
                    if (!(*it)->isHardwareTestComplete())
                    {
                        (*it)->setHardwareTestsComplete();
                    }
                    break;
                }
            }
            break;
    }

    int numSubsystems = drivers->commandScheduler.subsystemListSize();
    if (selectedSubsystem < 0)
    {
        selectedSubsystem = 0;
    }
    else if (selectedSubsystem >= numSubsystems)
    {
        selectedSubsystem = numSubsystems - 1;
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
    aruwlib::control::subsystem_scheduler_bitmap_t changedSubsystems = 0;

    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](control::Subsystem* sub) {
            changedSubsystems += (sub->isHardwareTestComplete() ? 1UL : 0UL) << i;
            i++;
        });

    changed |= (changedSubsystems != completeSubsystems);
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

    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](control::Subsystem* sub) {
            if (subsystemIndex <= topIndex && subsystemIndex >= bottomIndex)
                display << ((subsystemIndex == selectedSubsystem) ? ">" : " ")
                        << (sub->isHardwareTestComplete() ? "[done] " : "[not]  ") << sub->getName()
                        << modm::endl;
            subsystemIndex++;
        });
}
}  // namespace display
}  // namespace aruwlib
