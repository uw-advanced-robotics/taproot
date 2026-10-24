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

#ifndef TAPROOT_TRIGGER_HELPERS_HPP_
#define TAPROOT_TRIGGER_HELPERS_HPP_

#include <cstdlib>
#include <functional>

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"
#include "tap/control/trigger.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
using namespace tap::communication::serial;

/**
 * Helper struct that wraps different remote mappings in helper functions.
 */
struct TriggerHelpers
{
    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] button A remote button.
     * @return A trigger that is true when `button` is pressed on the remote.
     */
    static Trigger button(Drivers *drivers, Remote::Key button)
    {
        return Trigger(drivers, [drivers, button]() { return drivers->remote.keyPressed(button); });
    }

    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] remoteSwitch A remote switch (left/right).
     * @param[in] switchState A remote switch state (down/mid/up).
     * @return A trigger that is true when `remoteSwitch` is in state `switchState`.
     */
    static Trigger switchState(
        Drivers *drivers,
        Remote::Switch remoteSwitch,
        Remote::SwitchState switchState)
    {
        return Trigger(drivers, [drivers, remoteSwitch, switchState]() {
            return drivers->remote.getSwitch(remoteSwitch) == switchState;
        });
    }

    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] channel A channel on the remote.
     * @param[in] thresh A threshold for the value of `channel` on the remote.
     * @param[in] abs Whether to take the absolute value of `channel`, defaults to true.
     * @return A trigger that is true when the value of `channel` is greater than `thresh`.
     */
    static Trigger channelGreaterThan(
        Drivers *drivers,
        Remote::Channel channel,
        float thresh,
        bool abs = false)
    {
        return Trigger(drivers, [drivers, channel, thresh, abs]() {
            if (abs)
            {
                return std::abs(drivers->remote.getChannel(channel)) > thresh;
            }
            else
            {
                return drivers->remote.getChannel(channel) > thresh;
            }
        });
    }

    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @return A trigger that is true when the left mouse button is pressed.
     */
    static Trigger leftMouseButton(Drivers *drivers)
    {
        return Trigger(drivers, [drivers]() { return drivers->remote.getMouseL(); });
    }

    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @return A trigger that is true when the right mouse button is pressed.
     */
    static Trigger rightMouseButton(Drivers *drivers)
    {
        return Trigger(drivers, [drivers]() { return drivers->remote.getMouseR(); });
    }

    /**
     * Mouse x is the horizontal velocity of the mouse (positive is right), not a button.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse x velocity.
     * @param[in] abs Whether to take the absolute value of the mouse x velocity, defaults to
     * false.
     * @return A trigger that is true when the mouse x velocity is greater than `thresh`.
     */
    static Trigger mouseXGreaterThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseX, thresh, abs, true);
    }

    /**
     * Mouse x is the horizontal velocity of the mouse (positive is right), not a button. Use
     * a negative `thresh` to check for movement to the left, e.g. `thresh = -5` is true when
     * the mouse moves left faster than 5.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse x velocity.
     * @param[in] abs Whether to take the absolute value of the mouse x velocity, defaults to
     * false. When true, the trigger is true when the mouse x speed is below `thresh`.
     * @return A trigger that is true when the mouse x velocity is less than `thresh`.
     */
    static Trigger mouseXLessThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseX, thresh, abs, false);
    }

    /**
     * Mouse y is the vertical velocity of the mouse, not a button.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse y velocity.
     * @param[in] abs Whether to take the absolute value of the mouse y velocity, defaults to
     * false.
     * @return A trigger that is true when the mouse y velocity is greater than `thresh`.
     */
    static Trigger mouseYGreaterThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseY, thresh, abs, true);
    }

    /**
     * Mouse y is the vertical velocity of the mouse, not a button. Use a negative `thresh` to
     * check for movement in the negative y direction.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse y velocity.
     * @param[in] abs Whether to take the absolute value of the mouse y velocity, defaults to
     * false. When true, the trigger is true when the mouse y speed is below `thresh`.
     * @return A trigger that is true when the mouse y velocity is less than `thresh`.
     */
    static Trigger mouseYLessThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseY, thresh, abs, false);
    }

    /**
     * Mouse z is the scroll wheel velocity, not a button.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse z (scroll wheel) velocity.
     * @param[in] abs Whether to take the absolute value of the mouse z velocity, defaults to
     * false.
     * @return A trigger that is true when the mouse z velocity is greater than `thresh`.
     */
    static Trigger mouseZGreaterThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseZ, thresh, abs, true);
    }

    /**
     * Mouse z is the scroll wheel velocity, not a button. Use a negative `thresh` to check for
     * scrolling in the negative direction.
     *
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] thresh A threshold for the mouse z (scroll wheel) velocity.
     * @param[in] abs Whether to take the absolute value of the mouse z velocity, defaults to
     * false. When true, the trigger is true when the mouse z speed is below `thresh`.
     * @return A trigger that is true when the mouse z velocity is less than `thresh`.
     */
    static Trigger mouseZLessThan(Drivers *drivers, int16_t thresh, bool abs = false)
    {
        return mouseCompare(drivers, &Remote::getMouseZ, thresh, abs, false);
    }

private:
    using MouseAxisGetter = int16_t (Remote::*)() const;

    /**
     * @param[in] drivers A pointer to the drivers singleton.
     * @param[in] getter The `Remote` function that reads the mouse axis to compare.
     * @param[in] thresh A threshold for the value of the mouse axis.
     * @param[in] abs Whether to take the absolute value of the mouse axis.
     * @param[in] greater Whether to check that the value is greater than (true) or less than
     * (false) `thresh`.
     * @return A trigger that compares the mouse axis against `thresh`.
     */
    static Trigger mouseCompare(
        Drivers *drivers,
        MouseAxisGetter getter,
        int16_t thresh,
        bool abs,
        bool greater)
    {
        return Trigger(drivers, [drivers, getter, thresh, abs, greater]() {
            int value = (drivers->remote.*getter)();
            if (abs)
            {
                value = std::abs(value);
            }
            return greater ? value > thresh : value < thresh;
        });
    }
};
}  // namespace control
}  // namespace tap

#endif