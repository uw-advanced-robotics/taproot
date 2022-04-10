/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IMU_MENU_HPP_
#define TAPROOT_IMU_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/display/dummy_allocator.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "imu_interface.hpp"

namespace tap::communication::sensors::imu
{
/**
 * Menu that displays IMU readings from some particular `ImuInterface`.
 */
class ImuMenu : public modm::AbstractMenu<display::DummyAllocator<modm::IAbstractView> >
{
public:
    /** Time in milliseconds between calls to update the display. */
    static constexpr uint32_t IMU_UPDATE_TIME = 500;

    /**
     * @param[in] stack The `ViewStack` that this menu will is being added to.
     * @param[in] imu The ImuInterface whose IMU information will be displayed.
     */
    ImuMenu(
        modm::ViewStack<display::DummyAllocator<modm::IAbstractView> > *stack,
        ImuInterface *imu);

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    /**
     * @return The name of the menu, which happens to be the name of the IMU associated with this
     * menu.
     */
    const char *getMenuName();

private:
    using ImuInterfaceFnPtr = float (ImuInterface::*)();

    static constexpr int IMU_DATA_START_X = 20;
    static constexpr int IMU_DATA_START_Y = 10;

    static constexpr const char *IMU_DATA_COL_HEADERS[] = {"X", "Y", "Z"};

    ImuInterface *imu;

    tap::arch::PeriodicMilliTimer imuUpdateTimer{IMU_UPDATE_TIME};

    ImuInterfaceFnPtr imuAccelGyroAngleFnPtrs[3][3];
};
}  // namespace tap::communication::sensors::imu

#endif  // IMU_MENU_HPP_
