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

#ifndef TOW_SUBSYSTEM_HPP_
#define TOW_SUBSYSTEM_HPP_

#include <aruwlib/communication/gpio/digital.hpp>
#include <aruwlib/control/subsystem.hpp>

#include "mock_macros.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * A subsystem that houses an interface to communciate with the tow subsystem. The subsystem
 * includes the following:
 * - Two "clamps," each controlled by solenoids (controlled by GPIO digital out).
 * - Each clamp has a limit switch associated with it that when triggered indicates
 *   there is a robot in the grasp of the clamp.
 */
class TowSubsystem : public aruwlib::control::Subsystem
{
public:
    TowSubsystem(
        aruwlib::Drivers *drivers,
        aruwlib::gpio::Digital::OutputPin leftTowPin,
        aruwlib::gpio::Digital::OutputPin rightTowPin,
        aruwlib::gpio::Digital::InputPin leftTowLimitSwitchPin,
        aruwlib::gpio::Digital::InputPin rightTowLimitSwitchPin);

    /**
     * @param[in] isClamped Indicates if the left clamp should be clamped or not clamped.
     */
    mockable void setLeftClamped(bool isClamped);

    /**
     * @return `true` if the left clamp is clamped, `false` otherwise.
     */
    mockable bool getLeftClamped() const;

    /**
     * @param[in] isClamped Indicates if the right clamp should be clamped or not clamped.
     */
    mockable void setRightClamped(bool isClamped);

    /**
     * @return `true` if the right clamp is clamped, `false` otherwise.
     */
    mockable bool getRightClamped() const;

    /**
     * @return `true` if the limit switch associated with the left clamp has been triggered.
     */
    mockable bool getLeftLimitSwitchTriggered() const;

    /**
     * @return `true` if the limit switch associated with the left clamp has been triggered.
     */
    mockable bool getRightLeftLimitSwitchTriggered() const;

    void refresh() override {}

    void runHardwareTests() override;

    const char *getName() override { return "Tow Subsystem"; }

private:
    /**
     * Keeps track of the state of the subsystem - if the tower clamp is open or not.
     */
    bool leftClamped;
    bool rightClamped;

    const aruwlib::gpio::Digital::OutputPin LEFT_TOW_PIN;
    const aruwlib::gpio::Digital::OutputPin RIGHT_TOW_PIN;
    const aruwlib::gpio::Digital::InputPin LEFT_TOW_LIMIT_SWITCH;
    const aruwlib::gpio::Digital::InputPin RIGHT_TOW_LIMIT_SWITCH_PIN;
};  // class TowSubsystem
}  // namespace engineer
}  // namespace aruwsrc

#endif  // TOW_SUBSYSTEM_HPP_
