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

#ifndef IMU_MENU_HPP_
#define IMU_MENU_HPP_

#include "modm/ui/menu/abstract_menu.hpp"
#include "imu_interface.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

namespace tap::sensors::imu {
class ImuMenu : public modm::AbstractMenu<display::DummyAllocator<modm::IAbstractView> > {
public:
    static constexpr uint8_t IMU_MENU_ID = 10;
    static constexpr uint32_t IMU_UPDATE_TIME = 500;

    ImuMenu(modm::ViewStack<display::DummyAllocator<modm::IAbstractView> > *stack, ImuInterface *imu);

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char *getMenuName() { return "IMU Menu"; }

private:
    ImuInterface *imu;

    tap::arch::PeriodicMilliTimer imuUpdateTimer{IMU_UPDATE_TIME};
};
}

#endif  // IMU_MENU_HPP_
