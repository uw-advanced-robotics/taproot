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

#include <gtest/gtest.h>

#include "tap/control/trigger_helpers.hpp"
#include "tap/drivers.hpp"

using namespace tap::control;
using tap::Drivers;
using namespace testing;

TEST(TriggerHelpers, mouseX_greater_than_true_only_above_thresh)
{
    Drivers drivers;
    int16_t x = 0;
    ON_CALL(drivers.remote, getMouseX).WillByDefault(ReturnPointee(&x));

    Trigger trigger = TriggerHelpers::mouseX(&drivers, 50);

    EXPECT_FALSE(trigger.get());

    x = 50;
    EXPECT_FALSE(trigger.get());

    x = 51;
    EXPECT_TRUE(trigger.get());

    x = -60;
    EXPECT_FALSE(trigger.get());
}

TEST(TriggerHelpers, mouseX_greater_than_abs_true_in_either_direction)
{
    Drivers drivers;
    int16_t x = 0;
    ON_CALL(drivers.remote, getMouseX).WillByDefault(ReturnPointee(&x));

    Trigger trigger = TriggerHelpers::mouseX(&drivers, 50, true, true);

    EXPECT_FALSE(trigger.get());

    x = 60;
    EXPECT_TRUE(trigger.get());

    x = -60;
    EXPECT_TRUE(trigger.get());

    x = INT16_MIN;
    EXPECT_TRUE(trigger.get());
}

TEST(TriggerHelpers, mouseX_less_than_negative_thresh_true_only_moving_left)
{
    Drivers drivers;
    int16_t x = 0;
    ON_CALL(drivers.remote, getMouseX).WillByDefault(ReturnPointee(&x));

    Trigger trigger = TriggerHelpers::mouseX(&drivers, -5, false);

    EXPECT_FALSE(trigger.get());

    x = 60;
    EXPECT_FALSE(trigger.get());

    x = -5;
    EXPECT_FALSE(trigger.get());

    x = -6;
    EXPECT_TRUE(trigger.get());
}

TEST(TriggerHelpers, mouseX_less_than_abs_true_when_slow)
{
    Drivers drivers;
    int16_t x = 0;
    ON_CALL(drivers.remote, getMouseX).WillByDefault(ReturnPointee(&x));

    Trigger trigger = TriggerHelpers::mouseX(&drivers, 5, false, true);

    EXPECT_TRUE(trigger.get());

    x = -4;
    EXPECT_TRUE(trigger.get());

    x = 5;
    EXPECT_FALSE(trigger.get());

    x = -5;
    EXPECT_FALSE(trigger.get());
}

TEST(TriggerHelpers, mouseY_reads_mouse_y)
{
    Drivers drivers;
    int16_t y = 0;
    ON_CALL(drivers.remote, getMouseY).WillByDefault(ReturnPointee(&y));

    Trigger greater = TriggerHelpers::mouseY(&drivers, 5);
    Trigger less = TriggerHelpers::mouseY(&drivers, -5, false);

    EXPECT_FALSE(greater.get());
    EXPECT_FALSE(less.get());

    y = 10;
    EXPECT_TRUE(greater.get());
    EXPECT_FALSE(less.get());

    y = -10;
    EXPECT_FALSE(greater.get());
    EXPECT_TRUE(less.get());
}

TEST(TriggerHelpers, mouseZ_reads_scroll_direction)
{
    Drivers drivers;
    int16_t z = 0;
    ON_CALL(drivers.remote, getMouseZ).WillByDefault(ReturnPointee(&z));

    Trigger scrollUp = TriggerHelpers::mouseZ(&drivers, 0);
    Trigger scrollDown = TriggerHelpers::mouseZ(&drivers, 0, false);

    EXPECT_FALSE(scrollUp.get());
    EXPECT_FALSE(scrollDown.get());

    z = 1;
    EXPECT_TRUE(scrollUp.get());
    EXPECT_FALSE(scrollDown.get());

    z = -1;
    EXPECT_FALSE(scrollUp.get());
    EXPECT_TRUE(scrollDown.get());
}
